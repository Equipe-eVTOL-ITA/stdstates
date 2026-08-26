#pragma once

// TakeoffState — decola usando a abordagem escolhida no YAML.
//
// Este estado era a subida por setpoint. Agora ele e a CASCA: arma, reancora o
// referencial quando for o caso, le `takeoff_mode` da blackboard e delega. A
// subida por setpoint esta em takeoff/stdtakeoff.hpp, movida para la sem
// mudanca de comportamento.
//
//   opcional  "takeoff_mode"  std::string  stdtakeoff (padrao) | px4
//   os demais parametros dependem do modo -- ver cada header em takeoff/
//
// Outcomes: "" / "TAKEOFF COMPLETED" / "ERROR". Vocabulario fixo; ver
// takeoff/estrategia.hpp.
//
// O QUE NAO E DA ESTRATEGIA: armar e reancorar o referencial. Os dois valem
// para qualquer abordagem e tem ordem propria -- reancorar so depois de armar,
// para o EKF ja ter convergido no heading verdadeiro.

#include <Eigen/Eigen>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

#include "stdstates/blackboard_params.hpp"
#include "stdstates/takeoff/registro.hpp"

// >>> CONTRATO px4.reancoragem-do-home
// setHomePosition() REANCORA o referencial do mundo na posicao e no yaw atuais.
// Use TakeoffState(true) SO na decolagem inicial da missao.
//
// Numa redecolagem no meio da missao ela e destrutiva: a origem do mundo pula
// para onde o drone estiver, e tudo o que estava guardado em coordenadas de
// mundo -- bases ja visitadas, a grade de varredura, a posicao de casa --
// passa a se referir a um referencial que nao existe mais.
//
// Nao ha erro. O drone decola, olha para baixo, ve a base em que acabou de
// pousar, nao a reconhece, e pousa nela de novo. E de novo.
//
// Medido em SITL: o NED cru do PX4 ficou em (3.417, -0.159) o ciclo inteiro --
// o drone nunca saiu do lugar -- enquanto o FRD visto pela missao saltava de
// (-0.16, -3.03) para (0.00, 0.03) a cada redecolagem.
//
// Vale para as DUAS abordagens de decolagem: e do estado, e nao da estrategia.
// <<< CONTRATO

class TakeoffState : public fsm::State
{
public:
  /// @param set_home  Se true, reancora o referencial FRD na posicao atual.
  ///                  Verdadeiro para a decolagem INICIAL; FALSO para qualquer
  ///                  redecolagem. Ver o contrato acima.
  explicit TakeoffState(bool set_home = true)
  : fsm::State(), set_home_(set_home) {}

  /// Fixa a abordagem no codigo, ignorando o `takeoff_mode` do YAML.
  TakeoffState(bool set_home, std::string modo_fixo)
  : fsm::State(), set_home_(set_home), modo_fixo_(std::move(modo_fixo)) {}

  void on_enter(fsm::Blackboard & blackboard) override
  {
    estrategia_.reset();

    auto drone_ptr = blackboard.get<std::shared_ptr<Drone>>("drone");
    if (drone_ptr == nullptr) {return;}
    drone_ = *drone_ptr;
    if (drone_ == nullptr) {return;}

    drone_->log("");
    drone_->log("STATE: TAKEOFF");

    if (drone_->getArmingState() != DronePX4::ARMING_STATE::ARMED) {
      drone_->toOffboardSync();
      drone_->armSync();
    }

    // Depois de armar, de proposito: antes, o EKF ainda nao convergiu para o
    // heading verdadeiro e o initial_yaw_ fica em 0 enquanto o rumo real e
    // outro -- o primeiro setpoint de posicao produz uma guinada inesperada.
    if (set_home_) {
      drone_->setHomePosition(Eigen::Vector3d(0, 0, 0));
    }

    std::string modo = modo_fixo_;
    if (modo.empty()) {
      modo = stdstates::optional<std::string>(
        blackboard, "takeoff_mode", stdstates::takeoff::kModoPadrao);
    }

    auto estrategia = stdstates::takeoff::criar(modo);
    if (estrategia == nullptr) {
      drone_->log("ERRO: takeoff_mode desconhecido: '" + modo + "'.");
      std::string lista;
      for (const auto & m : stdstates::takeoff::modos()) {
        lista += (lista.empty() ? "" : ", ") + m;
      }
      drone_->log("      Modos aceitos: " + lista);
      return;
    }

    if (!estrategia->preparar(blackboard, drone_)) {return;}

    estrategia_ = std::move(estrategia);
    start_time_ = std::chrono::steady_clock::now();
  }

  std::string act(fsm::Blackboard & blackboard) override
  {
    (void)blackboard;
    if (estrategia_ == nullptr) {return "ERROR";}

    const std::chrono::duration<double> elapsed =
      std::chrono::steady_clock::now() - start_time_;

    return estrategia_->passo(drone_, elapsed.count());
  }

  void on_exit(fsm::Blackboard & blackboard) override
  {
    (void)blackboard;
    // A do PX4 reentra em offboard aqui: sem isso o estado seguinte manda
    // setpoint em AUTO_LOITER e o firmware o ignora em silencio.
    if (estrategia_ != nullptr && drone_ != nullptr) {
      estrategia_->encerrar(drone_);
    }
  }

private:
  bool set_home_;
  std::string modo_fixo_;
  std::shared_ptr<Drone> drone_;
  std::unique_ptr<stdstates::takeoff::Estrategia> estrategia_;
  std::chrono::steady_clock::time_point start_time_;
};
