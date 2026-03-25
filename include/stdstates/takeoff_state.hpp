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
 *   - "takeoff_height"         (float) — target altitude
 *   - "max_vertical_velocity"  (float) — velocity clamp
 *   - "position_tolerance"     (float) — distance to goal to consider done
 *
 * Returns:
 *   - "TAKEOFF COMPLETED" when the drone reaches the target altitude
 *   - "" while still climbing
 *
 * Behavior:
 *   1. Arms and switches to offboard mode if not already armed
 *   2. Moves toward the target altitude at clamped velocity
 *   3. Maintains initial XY position and yaw
 */
class TakeoffState : public fsm::State {
public:
    TakeoffState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone_ == nullptr) return;

        drone_->log("");
        drone_->log("STATE: TAKEOFF");

        max_velocity_ = *blackboard.get<float>("max_vertical_velocity");
        position_tolerance_ = *blackboard.get<float>("position_tolerance");
        float takeoff_height = *blackboard.get<float>("takeoff_height");

        pos_ = drone_->getLocalPosition();
        initial_yaw_ = drone_->getOrientation()[2];
        goal_ = Eigen::Vector3d(pos_[0], pos_[1], takeoff_height);

        if (drone_->getArmingState() != DronePX4::ARMING_STATE::ARMED) {
            drone_->toOffboardSync();
            drone_->armSync();
        }

        print_counter_ = 0;
        drone_->log("Initial Yaw: " + std::to_string(initial_yaw_));
        drone_->log("Takeoff at: " + std::to_string(pos_[0])
                    + " " + std::to_string(pos_[1]) + " " + std::to_string(pos_[2]));
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        if (drone_ == nullptr) return "ERROR";

        // Re-arm if the drone somehow disarmed
        if (drone_->getArmingState() != DronePX4::ARMING_STATE::ARMED) {
            drone_->log("Drone is not armed, arming again.");
            drone_->toOffboardSync();
            drone_->armSync();
        }

        // Periodic position logging
        if (print_counter_ % 10 == 0) {
            drone_->log("Pos: {" + std::to_string(pos_[0]) + ", "
                + std::to_string(pos_[1]) + ", " + std::to_string(pos_[2]) + "}");
        }
        print_counter_++;

        pos_ = drone_->getLocalPosition();
        Eigen::Vector3d diff = goal_ - pos_;

        if (diff.norm() < position_tolerance_) {
            drone_->log("Takeoff completed at altitude " + std::to_string(pos_[2]));
            return "TAKEOFF COMPLETED";
        }

        // Move toward goal with velocity clamping
        Eigen::Vector3d little_goal = pos_ + (diff.norm() > max_velocity_
                                              ? diff.normalized() * max_velocity_
                                              : diff);

        drone_->setLocalPosition(
            little_goal.x(),
            little_goal.y(),
            little_goal.z(),
            initial_yaw_);

        return "";
    }

private:
    std::shared_ptr<Drone> drone_;
    Eigen::Vector3d pos_, goal_;
    float max_velocity_;
    float initial_yaw_;
    float position_tolerance_;
    int print_counter_;
};