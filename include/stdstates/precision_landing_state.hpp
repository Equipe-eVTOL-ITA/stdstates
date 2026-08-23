#pragma once

// PrecisionLandingState — a casca: le `landing_mode` da blackboard, cria a
// estrategia e delega. A matematica esta em landing/.
//
// O nome da classe nao muda: e com ele que o stdbt registra o estado na fabrica
// da Behavior Tree, e ele aparece nos XML das arvores de outro repositorio.
//
//   opcional  "landing_mode"  std::string  exponencial (padrao) | px4 | s_curve
//   os demais parametros dependem do modo -- ver cada header em landing/
//
// Outcomes: "" / "LANDED" / "ERROR". Vocabulario fixo; ver estrategia.hpp.
//
// Nao faz contabilidade de base: um estado generico de pouso nao sabe o que e
// uma "base". A missao faz isso no estado seguinte.

#include <chrono>
#include <memory>
#include <string>
#include <utility>   // std::move, no construtor de modo fixo

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

#include "stdstates/blackboard_params.hpp"
#include "stdstates/landing/registro.hpp"

class PrecisionLandingState : public fsm::State
{
public:
  PrecisionLandingState()
  : fsm::State() {}

  /// Fixa a abordagem no codigo, ignorando o YAML -- para a fase cuja regra
  /// exige um pouso especifico. Sem argumento, o YAML manda.
  explicit PrecisionLandingState(std::string modo_fixo)
  : fsm::State(), modo_fixo_(std::move(modo_fixo)) {}

  void on_enter(fsm::Blackboard & blackboard) override
  {
    estrategia_.reset();

    auto drone_ptr = blackboard.get<std::shared_ptr<Drone>>("drone");
    if (drone_ptr == nullptr) {return;}
    drone_ = *drone_ptr;
    if (drone_ == nullptr) {return;}

    drone_->log("");
    drone_->log("STATE: PRECISION LANDING");

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
    start_time_ = std::chrono::steady_clock::now();
  }

  std::string act(fsm::Blackboard & blackboard) override
  {
    (void)blackboard;
    if (estrategia_ == nullptr) {return "ERROR";}

    // Em segundos FRACIONARIOS: truncar para inteiro fazia da exponencial uma
    // escada de um degrau por segundo.
    const std::chrono::duration<double> elapsed =
      std::chrono::steady_clock::now() - start_time_;

    return estrategia_->passo(drone_, elapsed.count());
  }

  void on_exit(fsm::Blackboard & blackboard) override
  {
    (void)blackboard;
    // Roda por qualquer caminho de saida. Cada estrategia sabe como parar.
    if (estrategia_ != nullptr && drone_ != nullptr) {
      estrategia_->encerrar(drone_);
    }
  }

private:
  std::shared_ptr<Drone> drone_;
  std::unique_ptr<stdstates::landing::Estrategia> estrategia_;
  std::string modo_fixo_;
  std::chrono::steady_clock::time_point start_time_;
};
