#pragma once

#include <Eigen/Eigen>
#include <cstdint>
#include <memory>
#include <string>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

/**
 * @brief Standard takeoff state for FSM-based drone missions.
 *
 * Reads from blackboard:
 *   - "drone"                  (std::shared_ptr<Drone>) — drone instance
 *
 * Returns:
 *   - "ARMED" when the drone is armed
 *   - "" while still trying to arm it
 *
 * Behavior:
 *   1. Arms and switches to offboard mode if not already armed
 */
class ArmingState : public fsm::State {
public:
    ArmingState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone_ == nullptr) return;

        drone_->log("");
        drone_->log("STATE: ARMING");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        if (drone_ == nullptr) return "ERROR";

        if (drone_->getArmingState() != DronePX4::ARMING_STATE::ARMED) {
            drone_->log("Drone is not armed, arming again.");
            drone_->toOffboardSync();
            drone_->armSync();
            return "";
        }

        return "ARMED";
    }

private:
    std::shared_ptr<Drone> drone_;
};