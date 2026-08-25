#pragma once

// LandAndDisarmState — pousa pela abordagem escolhida e GARANTE o desarme,
// sem bloquear.
//
// A descida e delegada a mesma estrategia que o PrecisionLandingState usa, e
// escolhida pela mesma chave: `landing_mode`. Antes este estado era fixo no
// modo LAND do PX4, entao uma missao com `landing_mode: s_curve` pousava em S
// no meio da missao e em LAND no fim -- duas abordagens diferentes no mesmo
// voo, sem ninguem ter pedido.
//
// O que ele acrescenta ao PrecisionLandingState e a segunda fase: depois de a
// estrategia terminar a descida, ele insiste no desarme ate confirmar. Para
// `landing_mode: px4` isso e redundante (o firmware ja desarma) e sai de graca:
// a fase de desarme ve o drone ja desarmado no primeiro tick.
//
// Em 2025 isto vivia no `on_exit` do ReturnHome, com `disarmSync()` e um
// `rclcpp::sleep_for(5s)`. O `on_exit` roda DENTRO do callback do timer da FSM:
// bloquear ali congela o executor inteiro -- a telemetria para, a visao para, e
// o no fica sem responder justamente quando alguem no chao olha para a tela
// para saber se o drone pousou. Aqui a espera e estado, e a FSM continua
// girando.
//
// Contrato com a blackboard:
//
//   opcional  "landing_mode"    std::string  exponencial (padrao) | px4 | s_curve
//   opcional  "disarm_grace"    float  s antes de forcar o desarme (padrao 3)
//   opcional  "disarm_timeout"  float  s ate desistir (padrao 20)
//   os demais parametros dependem do modo -- ver stdstates/landing/
//
// Outcomes: ""          (descendo ou esperando desarmar)
//           "DISARMED"  (confirmado desarmado)
//           "TIMEOUT"   (nao desarmou a tempo)
//           "ERROR"     (parametro ausente, incoerente, ou modo desconhecido)
//
// TIMEOUT e outcome proprio de proposito: devolver "DISARMED" com o drone
// ainda armado seria declarar sucesso sem ter havido.

#include <chrono>
#include <memory>
#include <string>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

#include "stdstates/blackboard_params.hpp"
#include "stdstates/landing/registro.hpp"

class LandAndDisarmState : public fsm::State
{
public:
  LandAndDisarmState()
  : fsm::State() {}

  /// Fixa a abordagem no codigo, ignorando o `landing_mode` do YAML.
  explicit LandAndDisarmState(std::string modo_fixo)
  : fsm::State(), modo_fixo_(std::move(modo_fixo)) {}

  void on_enter(fsm::Blackboard & blackboard) override
  {
    estrategia_.reset();
    pediu_desarme_ = false;
    fase_ = Fase::Descendo;

    auto drone_ptr = blackboard.get<std::shared_ptr<Drone>>("drone");
    if (drone_ptr == nullptr) {return;}
    drone_ = *drone_ptr;
    if (drone_ == nullptr) {return;}

    drone_->log("");
    drone_->log("STATE: LAND AND DISARM");

    grace_s_ = stdstates::optional<float>(blackboard, "disarm_grace", 3.0f);
    timeout_s_ = stdstates::optional<float>(blackboard, "disarm_timeout", 20.0f);

    std::string modo = modo_fixo_;
    if (modo.empty()) {
      modo = stdstates::optional<std::string>(
        blackboard, "landing_mode", stdstates::landing::kModoPadrao);
    }

    auto estrategia = stdstates::landing::criar(modo);
    if (estrategia == nullptr) {
      drone_->log("ERRO: landing_mode desconhecido: '" + modo + "'.");
      std::string lista;
      for (const auto & m : stdstates::landing::modos()) {
        lista += (lista.empty() ? "" : ", ") + m;
      }
      drone_->log("      Modos aceitos: " + lista);
      return;
    }

    if (!estrategia->preparar(blackboard, drone_)) {return;}

    estrategia_ = std::move(estrategia);
    inicio_ = std::chrono::steady_clock::now();
  }

  std::string act(fsm::Blackboard & blackboard) override
  {
    (void)blackboard;
    if (estrategia_ == nullptr) {return "ERROR";}

    if (drone_->getArmingState() == DronePX4::ARMING_STATE::DISARMED) {
      drone_->log("Desarmado.");
      return "DISARMED";
    }

    const std::chrono::duration<double> desde_inicio =
      std::chrono::steady_clock::now() - inicio_;

    if (fase_ == Fase::Descendo) {
      const std::string r = estrategia_->passo(drone_, desde_inicio.count());

      if (r == stdstates::landing::kErro) {
        // A estrategia desistiu com o drone ainda armado. Para o modo px4 isso
        // e o proprio timeout de desarme dele -- e "nao desarmou", que e
        // TIMEOUT, e nao um erro de configuracao.
        drone_->log("AVISO: a estrategia de pouso desistiu com o drone armado.");
        return "TIMEOUT";
      }

      if (r != stdstates::landing::kPousado) {return "";}

      // Desceu. Para o movimento e passa a insistir no desarme.
      estrategia_->encerrar(drone_);
      fase_ = Fase::Desarmando;
      inicio_desarme_ = std::chrono::steady_clock::now();
      return "";
    }

    const std::chrono::duration<double> desde_pouso =
      std::chrono::steady_clock::now() - inicio_desarme_;

    if (desde_pouso.count() > timeout_s_) {
      drone_->log(
        "AVISO: nao desarmou em " + std::to_string(timeout_s_) +
        " s. O drone pode continuar armado.");
      return "TIMEOUT";
    }

    // Passada a carencia, insiste. `disarm()` e o comando de um tiro;
    // `disarmSync()` seria o laco bloqueante que este estado existe para evitar.
    if (desde_pouso.count() > grace_s_) {
      if (!pediu_desarme_) {
        drone_->log("Nao desarmou sozinho; forcando.");
        pediu_desarme_ = true;
      }
      drone_->disarm();
    }

    return "";
  }

  void on_exit(fsm::Blackboard & blackboard) override
  {
    (void)blackboard;
    if (estrategia_ != nullptr && drone_ != nullptr) {
      estrategia_->encerrar(drone_);
    }
  }

private:
  enum class Fase { Descendo, Desarmando };

  std::shared_ptr<Drone> drone_;
  std::unique_ptr<stdstates::landing::Estrategia> estrategia_;
  std::string modo_fixo_;
  Fase fase_ = Fase::Descendo;
  float grace_s_ = 3.0f;
  float timeout_s_ = 20.0f;
  bool pediu_desarme_ = false;
  std::chrono::steady_clock::time_point inicio_;
  std::chrono::steady_clock::time_point inicio_desarme_;
};
