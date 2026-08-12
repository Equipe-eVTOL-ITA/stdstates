#pragma once

// WaypointListState — percorre uma lista de waypoints da blackboard.
//
// Extraído da parte de NAVEGAÇÃO do `search_base_state.hpp` da fase 1 da CBR
// 2025. O estado de lá misturava duas responsabilidades: andar pela grade e
// decidir se uma detecção era base nova. A segunda é regra de negócio da fase e
// fica na missão; só a primeira é genérica e vem para cá.
//
// Contrato com a blackboard:
//
//   entrada   "waypoints"           std::vector<ArenaPoint>  (a missão gera)
//             "position_tolerance"  float
//             "max_horizontal_velocity" float
//   opcional  "waypoint_yaw"        float  (padrão: yaw ao entrar no estado)
//
// Outcomes: ""  (ainda percorrendo)
//           "WAYPOINTS ENDED"  (todos visitados)
//           "ERROR"            (parâmetro ausente na blackboard)
//
// A missão que quiser interromper a varredura no meio — para ir até uma base
// detectada, por exemplo — deve derivar deste estado e devolver o seu próprio
// outcome antes de chamar a navegação. Ver `SearchBaseState` da cbr2026.

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Eigen>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

#include "stdstates/arena_point.hpp"
#include "stdstates/blackboard_params.hpp"

class WaypointListState : public fsm::State
{
public:
  WaypointListState()
  : fsm::State() {}

  void on_enter(fsm::Blackboard & blackboard) override
  {
    ok_ = false;
    waypoints_ = nullptr;

    auto drone_ptr = blackboard.get<std::shared_ptr<Drone>>("drone");
    if (drone_ptr == nullptr) return;
    drone_ = *drone_ptr;
    if (drone_ == nullptr) return;

    drone_->log("");
    drone_->log("STATE: WAYPOINT LIST");

    // A lista é lida por PONTEIRO, de propósito: o estado marca `is_visited` e
    // esse progresso tem de sobreviver a sair e voltar ao estado. Uma cópia
    // reiniciaria a varredura toda vez que a missão desviasse para outro estado.
    waypoints_ = blackboard.get<std::vector<ArenaPoint>>("waypoints");
    if (waypoints_ == nullptr) {
      drone_->log("ERRO: parametro ausente na blackboard: 'waypoints'");
      return;
    }

    if (!stdstates::require(blackboard, drone_, "position_tolerance", tolerance_)) return;
    if (!stdstates::require(blackboard, drone_, "max_horizontal_velocity", max_velocity_)) return;

    // Manter o yaw em que se entrou evita que o drone gire enquanto varre — o
    // que importa quando a câmera é fixa ao corpo e a varredura é para procurar
    // algo no chão.
    yaw_ = stdstates::optional<float>(
      blackboard, "waypoint_yaw", static_cast<float>(drone_->getOrientation()[2]));

    ok_ = true;
  }

  std::string act(fsm::Blackboard & blackboard) override
  {
    (void)blackboard;
    if (!ok_) return "ERROR";

    return navigate();
  }

protected:
  /// Avança um passo na rota. Separado de `act` para que uma missão possa
  /// derivar deste estado, rodar a sua própria lógica primeiro e só então
  /// chamar a navegação. É o que a fase 1 faz para interromper a varredura ao
  /// encontrar uma base nova.
  std::string navigate()
  {
    ArenaPoint * goal_point = nextUnvisited();
    if (goal_point == nullptr) {
      drone_->setLocalVelocity(0.0, 0.0, 0.0, 0.0);
      drone_->log("Todos os waypoints visitados.");
      return "WAYPOINTS ENDED";
    }

    const Eigen::Vector3d pos = drone_->getLocalPosition();
    const Eigen::Vector3d diff = goal_point->coordinates - pos;

    if (diff.norm() < tolerance_) {
      goal_point->is_visited = true;
      drone_->log(
        "Waypoint visitado: {" + std::to_string(pos.x()) + ", " +
        std::to_string(pos.y()) + ", " + std::to_string(pos.z()) + "}");
      return "";
    }

    // Passo de comprimento limitado em vez do alvo final: manda o drone para um
    // ponto próximo, o que faz o controlador do PX4 produzir uma velocidade
    // aproximadamente constante em vez de acelerar ao máximo em direção a um
    // alvo distante.
    const Eigen::Vector3d step =
      diff.norm() > max_velocity_ ? diff.normalized() * max_velocity_ : diff;
    const Eigen::Vector3d little_goal = pos + step;

    drone_->setLocalPosition(little_goal.x(), little_goal.y(), little_goal.z(), yaw_);
    return "";
  }

  /// Primeiro waypoint ainda não visitado, ou nullptr se acabaram.
  ArenaPoint * nextUnvisited()
  {
    if (waypoints_ == nullptr) return nullptr;
    for (auto & point : *waypoints_) {
      if (!point.is_visited) return &point;
    }
    return nullptr;
  }

  std::shared_ptr<Drone> drone_;
  std::vector<ArenaPoint> * waypoints_ = nullptr;
  float tolerance_ = 0.0f;
  float max_velocity_ = 0.0f;
  float yaw_ = 0.0f;
  bool ok_ = false;
};
