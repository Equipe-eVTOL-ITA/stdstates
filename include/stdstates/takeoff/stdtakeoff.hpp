#pragma once

// StdTakeoff — a subida por setpoint de posicao, que e a decolagem padrao.
//
// Sobe em passo constante ate `takeoff_height`, mantendo o XY e a guinada da
// decolagem. O drone segue em OFFBOARD o tempo todo -- quem controla e a
// missao, e o PX4 so obedece.
//
// Este arquivo e o miolo do TakeoffState, movido para ca SEM MUDANCA de
// comportamento: uma missao que nao declara `takeoff_mode` decola exatamente
// como decolava.
//
//   entrada  "takeoff_height"          float  (FRD, NEGATIVO = para cima)
//            "max_vertical_velocity"   float
//            "position_tolerance"      float

#include <memory>
#include <string>

#include <Eigen/Eigen>

#include "drone/movement.hpp"

#include "stdstates/blackboard_params.hpp"
#include "stdstates/takeoff/estrategia.hpp"

namespace stdstates::takeoff
{

class StdTakeoff : public Estrategia
{
public:
  const char * nome() const override {return "stdtakeoff";}

  bool preparar(fsm::Blackboard & blackboard, const std::shared_ptr<Drone> & drone) override
  {
    if (!stdstates::require(blackboard, drone, "max_vertical_velocity", max_velocity_)) {
      return false;
    }
    if (!stdstates::require(blackboard, drone, "position_tolerance", position_tolerance_)) {
      return false;
    }
    float takeoff_height = 0.0f;
    if (!stdstates::require(blackboard, drone, "takeoff_height", takeoff_height)) {return false;}

    const Eigen::Vector3d pos = drone->getLocalPosition();
    // O XY da decolagem e o de agora: sobe na vertical, sem deslocar.
    goal_ = Eigen::Vector3d(pos[0], pos[1], takeoff_height);
    initial_yaw_ = static_cast<float>(drone->getOrientation()[2]);
    print_counter_ = 0;

    drone->log(
      "Decolagem PADRAO ate " + std::to_string(takeoff_height) +
      " m (offboard, passo de " + std::to_string(max_velocity_) + " m)");
    return true;
  }

  std::string passo(const std::shared_ptr<Drone> & drone, double t) override
  {
    (void)t;

    // Rearma se o drone desarmou por algum motivo no meio da subida.
    if (drone->getArmingState() != DronePX4::ARMING_STATE::ARMED) {
      drone->log("Drone is not armed, arming again.");
      drone->toOffboardSync();
      drone->armSync();
    }

    const Eigen::Vector3d pos = drone->getLocalPosition();

    if (print_counter_ % 10 == 0) {
      drone->log(
        "Pos: {" + std::to_string(pos[0]) + ", " + std::to_string(pos[1]) +
        ", " + std::to_string(pos[2]) + "}");
    }
    print_counter_++;

    if ((goal_ - pos).norm() < position_tolerance_) {
      drone->log("Takeoff completed at altitude " + std::to_string(pos[2]));
      return kDecolou;
    }

    move_local_constant_step(drone, goal_, max_velocity_, position_tolerance_, initial_yaw_);
    return kSeguir;
  }

private:
  Eigen::Vector3d goal_ = Eigen::Vector3d::Zero();
  float max_velocity_ = 0.0f;
  float initial_yaw_ = 0.0f;
  float position_tolerance_ = 0.0f;
  int print_counter_ = 0;
};

}  // namespace stdstates::takeoff
