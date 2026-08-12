#pragma once

// LandAndDisarmState — pousa em modo LAND e desarma, SEM BLOQUEAR.
//
// Em 2025 isto vivia no `on_exit` do ReturnHome da fase 3:
//
//     this->drone->land();
//     this->drone->disarmSync();
//
// e na fase 1 era pior ainda, com um `rclcpp::sleep_for(5s)` no meio. O
// `on_exit` roda DENTRO do callback do timer da FSM, e `disarmSync()` é um laço
// `while (armado) { disarm(); usleep(100ms); }`. Bloquear ali congela o
// executor inteiro: a telemetria para, a visão para, e o nó fica sem responder
// justamente no momento em que alguém no chão está olhando para a tela para
// saber se o drone pousou.
//
// Aqui a espera vira estado, e a FSM continua girando.
//
// Contrato com a blackboard:
//
//   opcional  "disarm_grace"    float  s antes de forçar o desarme (padrão 3)
//   opcional  "disarm_timeout"  float  s até desistir (padrão 20)
//
// Outcomes: ""          (descendo ou esperando desarmar)
//           "DISARMED"  (confirmado desarmado)
//           "TIMEOUT"   (não desarmou a tempo)
//           "ERROR"
//
// TIMEOUT é outcome próprio de propósito. Devolver "DISARMED" quando o drone
// ainda está armado seria declarar sucesso sem ter havido — e a missão perde a
// chance de tratar o caso.

#include <chrono>
#include <memory>
#include <string>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

#include "stdstates/blackboard_params.hpp"

class LandAndDisarmState : public fsm::State
{
public:
  LandAndDisarmState()
  : fsm::State() {}

  void on_enter(fsm::Blackboard & blackboard) override
  {
    ok_ = false;
    pediu_desarme_ = false;

    auto drone_ptr = blackboard.get<std::shared_ptr<Drone>>("drone");
    if (drone_ptr == nullptr) return;
    drone_ = *drone_ptr;
    if (drone_ == nullptr) return;

    drone_->log("");
    drone_->log("STATE: LAND AND DISARM");

    grace_s_ = stdstates::optional<float>(blackboard, "disarm_grace", 3.0f);
    timeout_s_ = stdstates::optional<float>(blackboard, "disarm_timeout", 20.0f);

    // O modo LAND do PX4 desce e desarma sozinho ao detectar o solo. Pedir o
    // desarme antes disso pode ser recusado — daí a carência.
    drone_->land();
    inicio_ = std::chrono::steady_clock::now();

    ok_ = true;
  }

  std::string act(fsm::Blackboard & blackboard) override
  {
    (void)blackboard;
    if (!ok_) return "ERROR";

    if (drone_->getArmingState() == DronePX4::ARMING_STATE::DISARMED) {
      drone_->log("Desarmado.");
      return "DISARMED";
    }

    const std::chrono::duration<double> decorrido =
      std::chrono::steady_clock::now() - inicio_;

    if (decorrido.count() > timeout_s_) {
      drone_->log(
        "AVISO: nao desarmou em " + std::to_string(timeout_s_) +
        " s. O drone pode continuar armado.");
      return "TIMEOUT";
    }

    // Passada a carência, insiste no desarme. `disarm()` é o comando de um
    // tiro; `disarmSync()` seria o laço bloqueante que este estado existe para
    // não usar.
    if (decorrido.count() > grace_s_) {
      if (!pediu_desarme_) {
        drone_->log("Nao desarmou sozinho; forcando.");
        pediu_desarme_ = true;
      }
      drone_->disarm();
    }

    return "";
  }

private:
  std::shared_ptr<Drone> drone_;
  float grace_s_ = 3.0f;
  float timeout_s_ = 20.0f;
  bool pediu_desarme_ = false;
  std::chrono::steady_clock::time_point inicio_;
  bool ok_ = false;
};
