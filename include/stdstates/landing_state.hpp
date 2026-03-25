#pragma once

#include <Eigen/Eigen>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <memory>
#include <string>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

/**
 * @brief Standard landing state for FSM-based drone missions.
 *
 * Uses an exponential velocity decay profile for smooth landing.
 * Velocity starts at v_max and decays exponentially to v_min,
 * then maintains v_min until timeout.
 *
 * Reads from blackboard:
 *   - "drone"                 (std::shared_ptr<Drone>) — drone instance
 *   - "landing_velocity_max"  (float) — initial descent velocity
 *   - "landing_velocity_min"  (float) — minimum descent velocity
 *   - "max_base_height"       (float) — estimated ground/base height (positive value)
 *   - "landing_timeout"       (float) — extra seconds after estimated landing time
 *
 * Returns:
 *   - "LANDED" when the timeout is reached
 *   - "" while still descending
 *
 * Behavior:
 *   1. Computes a time constant for exponential velocity decay
 *   2. Descends at v(t) = v_max * exp(-k*t), clamped to [v_min, v_max]
 *   3. Stops after estimated landing time + timeout margin
 */
class LandingState : public fsm::State {
public:
    LandingState() : fsm::State() {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone_ == nullptr) return;

        drone_->log("");
        drone_->log("STATE: LANDING");

        v_max_ = *blackboard.get<float>("landing_velocity_max");
        v_min_ = *blackboard.get<float>("landing_velocity_min");

        // Get current drone height as starting point
        float current_height = drone_->getLocalPosition().z();
        float align_height = -current_height; // Current height (negative in FRD frame)
        float max_base_height = -*blackboard.get<float>("max_base_height"); // Negative in FRD

        drone_->log("Current height: " + std::to_string(align_height) + " m");

        // Compute exponential decay time constant
        time_constant_ = (v_max_ - v_min_) / (align_height - max_base_height);

        // Estimate total landing time
        double time_to_base = (1.0 / time_constant_) * std::log(v_max_ / v_min_);
        double total_time = time_to_base + max_base_height / v_min_;

        float landing_timeout_margin = *blackboard.get<float>("landing_timeout");
        timeout_ = total_time + landing_timeout_margin;
        start_time_ = std::chrono::steady_clock::now();

        drone_->log("Time to base: " + std::to_string(time_to_base) + " s");
        drone_->log("Total landing time: " + std::to_string(total_time) + " s");
    }

    std::string act(fsm::Blackboard &blackboard) override {
        (void)blackboard;

        if (drone_ == nullptr) return "ERROR";

        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(
            current_time - start_time_).count();

        // Exponential velocity decay
        float velocity = v_max_ * std::exp(-time_constant_ * elapsed_time);
        velocity = std::clamp(velocity, v_min_, v_max_);

        if (elapsed_time > timeout_) {
            drone_->setLocalVelocity(0.0, 0.0, 0.0, 0.0);
            drone_->log("Landing completed");
            return "LANDED";
        }

        drone_->setLocalVelocity(0.0, 0.0, velocity, 0.0);

        return "";
    }

private:
    std::shared_ptr<Drone> drone_;
    float v_max_, v_min_;
    float time_constant_;
    double timeout_;
    std::chrono::steady_clock::time_point start_time_;
};