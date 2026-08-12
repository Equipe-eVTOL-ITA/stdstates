#pragma once

// YawSweepState — gira em torno do próprio eixo, varrendo um setor.
//
// Extraído do `search_state.hpp` da fase 3 da CBR 2025. Lá o estado misturava
// duas responsabilidades: girar, e decidir se a mão do operador apareceu. A
// segunda é da missão; só a primeira é genérica e vem para cá.
//
// A missão que quiser parar de varrer ao encontrar algo deriva deste estado,
// roda a sua condição primeiro e só então chama `sweep()`. É o mesmo padrão de
// `SearchBaseState` sobre `WaypointListState` na fase 1.
//
// Contrato com a blackboard:
//
//   entrada   "yaw_speed"         float  rad/s
//             "search_yaw_range"  float  rad, meia-abertura do setor
//   opcional  "sweep_hold_z"      bool   manter a altitude atual (padrão: true)
//
// Outcomes: ""       (varrendo)
//           "ERROR"  (parâmetro ausente)

#include <cmath>
#include <memory>
#include <string>

#include <Eigen/Eigen>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/movement.hpp"

#include "stdstates/blackboard_params.hpp"

class YawSweepState : public fsm::State
{
public:
  YawSweepState()
  : fsm::State() {}

  void on_enter(fsm::Blackboard & blackboard) override
  {
    ok_ = false;
    sentido_ = +1.0f;

    auto drone_ptr = blackboard.get<std::shared_ptr<Drone>>("drone");
    if (drone_ptr == nullptr) return;
    drone_ = *drone_ptr;
    if (drone_ == nullptr) return;

    drone_->log("");
    drone_->log("STATE: YAW SWEEP");

    if (!stdstates::require(blackboard, drone_, "yaw_speed", yaw_speed_)) return;
    if (!stdstates::require(blackboard, drone_, "search_yaw_range", range_)) return;

    // O centro do setor é onde o drone está ao entrar. Guardar o ÂNGULO e
    // comparar por diferença normalizada, em vez de guardar limites absolutos,
    // é o que faz isto funcionar em qualquer orientação inicial.
    centro_ = static_cast<float>(drone_->getOrientation()[2]);

    ok_ = true;
  }

  std::string act(fsm::Blackboard & blackboard) override
  {
    (void)blackboard;
    if (!ok_) return "ERROR";

    sweep();
    return "";
  }

protected:
  /// Avança um passo do varrimento. Separado de `act` para que uma missão possa
  /// derivar deste estado, rodar a própria condição de parada primeiro e só
  /// então girar.
  void sweep()
  {
    const float yaw = static_cast<float>(drone_->getOrientation()[2]);

    // AQUI ESTÁ A CORREÇÃO DE 2025, e ela é a razão de este estado existir.
    //
    // A versão antiga guardava limites absolutos no on_enter:
    //
    //     min_yaw_ = yaw_atual - range;
    //     max_yaw_ = yaw_atual + range;
    //
    // e no act comparava `yaw >= max_yaw_` cru. O yaw do drone vive em
    // (-pi, pi]. Se o drone entra no estado olhando para 2.6 rad com uma
    // abertura de 1.05 rad, `max_yaw_` vale 3.65 — um valor que o yaw NUNCA
    // atinge, porque ao passar de pi ele salta para -pi. O drone gira sem
    // parar num sentido só, e ao cruzar a descontinuidade satisfaz
    // `yaw <= min_yaw_` de forma espúria e inverte no lugar errado.
    //
    // Comparar a DIFERENÇA normalizada em relação ao centro não tem esse
    // problema: ela é sempre o menor ângulo entre as duas direções, e vive em
    // (-pi, pi] por construção.
    const float desvio = normalizeYawError(yaw - centro_);

    if (desvio >= range_) {
      sentido_ = -1.0f;
    } else if (desvio <= -range_) {
      sentido_ = +1.0f;
    }

    drone_->setLocalVelocity(0.0, 0.0, 0.0, sentido_ * yaw_speed_);
  }

  std::shared_ptr<Drone> drone_;
  float yaw_speed_ = 0.0f;
  float range_ = 0.0f;
  float centro_ = 0.0f;
  float sentido_ = 1.0f;
  bool ok_ = false;
};
