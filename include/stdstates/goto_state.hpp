#pragma once

#include <Eigen/Eigen>
#include <cmath>
#include <memory>
#include <string>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

#include "stdstates/motion.hpp"

class GoToState : public fsm::State {
public:
    GoToState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone_ == nullptr) return;

        drone_->log("");
        drone_->log("STATE: GOTO");

        target_x_ = *blackboard.get<float>("target_x");
        target_y_ = *blackboard.get<float>("target_y");
        target_z_ = *blackboard.get<float>("target_z");
        target_yaw_ = *blackboard.get<float>("target_yaw");
        
        tolerance_ = *blackboard.get<float>("position_tolerance");

        // COMO chegar ao destino e escolha da missao. O padrao reproduz o que
        // este estado fazia antes; `motion_policy: axial` faz o drone girar e
        // so entao avancar.
        politica_ = stdstates::criarPolitica(blackboard, drone_);
        if (politica_ != nullptr) {
            limites_ = stdstates::limitesDaBlackboard(blackboard, tolerance_);
            politica_->iniciar(
                Eigen::Vector3d(target_x_, target_y_, target_z_), target_yaw_);
        }
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;
        if (drone_ == nullptr) return "ERROR";
        if (politica_ == nullptr) return "ERROR";

        Eigen::Vector3d pos = drone_->getLocalPosition();
        
        float dx = target_x_ - pos.x();
        float dy = target_y_ - pos.y();
        float dz = target_z_ - pos.z();
        
        float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        
        if (dist < tolerance_) {
            drone_->setLocalVelocity(0.0, 0.0, 0.0, 0.0);
            return "ARRIVED";
        }

        // Todo o deslocamento passa pela politica. Antes era um setpoint
        // direto no destino: em linha reta, e de lado sempre que o destino
        // estivesse de lado.
        politica_->irPara(
            drone_,
            Eigen::Vector3d(target_x_, target_y_, target_z_),
            target_yaw_, limites_);

        return "";
    }

private:
    std::shared_ptr<Drone> drone_;
    std::unique_ptr<drone::MotionPolicy> politica_;
    drone::Limites limites_;
    float target_x_, target_y_, target_z_, target_yaw_;
    float tolerance_;
};
