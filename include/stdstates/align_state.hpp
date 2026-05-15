#pragma once

#include <Eigen/Eigen>
#include <cmath>
#include <memory>
#include <string>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/PidController.hpp"
#include "drone/movement.hpp"
#include "drone/transformations.hpp"

class AlignState : public fsm::State {
public:
    AlignState(bool align_center = true, bool align_yaw = true) 
        : fsm::State(), align_center_(align_center), align_yaw_(align_yaw),
          pid_y_(0.0f, 0.0f, 0.0f, 0.0f), pid_yaw_(0.0f, 0.0f, 0.0f, 0.0f), target_yaw_(0.0f) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone_ == nullptr) return;

        drone_->log("");
        drone_->log("STATE: ALIGN");

        tolerance_y_ = *blackboard.get<float>("align_tolerance_y");
        tolerance_yaw_ = *blackboard.get<float>("align_tolerance_yaw");
        
        pid_y_ = PidController(
            *blackboard.get<float>("align_kp_y"), 
            *blackboard.get<float>("align_ki_y"), 
            *blackboard.get<float>("align_kd_y"), 
            0.0f);
            
        pid_yaw_ = PidController(
            *blackboard.get<float>("align_kp_yaw"), 
            *blackboard.get<float>("align_ki_yaw"), 
            *blackboard.get<float>("align_kd_yaw"), 
            0.0f);
            
        pid_y_.reset();
        pid_yaw_.reset();

        target_yaw_ = drone_->getOrientation()[2];
    }

    std::string act(fsm::Blackboard &blackboard) override {
        if (drone_ == nullptr) return "ERROR";

        float offset_y = 0.0f;
        float angle_err = 0.0f;

        if (blackboard.contains("hose_offset_y")) {
            offset_y = *blackboard.get<float>("hose_offset_y");
        }
        if (blackboard.contains("hose_angle_error")) {
            angle_err = *blackboard.get<float>("hose_angle_error");
        }

        bool aligned_y = !align_center_ || (std::abs(offset_y) < tolerance_y_);
        bool aligned_yaw = !align_yaw_ || (std::abs(angle_err) < tolerance_yaw_);

        if (aligned_y && aligned_yaw) {
            Eigen::Vector3d pos = drone_->getLocalPosition();
            drone_->setLocalPosition(pos.x(), pos.y(), pos.z(), drone_->getOrientation()[2]);
            drone_->log("Alignment complete");
            return "ALIGNED";
        }

        float vy = 0.0f;
        float rate_yaw = 0.0f;

        if (align_center_ && !aligned_y) {
            vy = pid_y_.compute(offset_y); 
        }
        if (align_yaw_ && !aligned_yaw) {
            rate_yaw = pid_yaw_.compute(angle_err);
        }

        Eigen::Vector3d local_delta(0.0, vy, 0.0);
        Eigen::Vector3d world_delta = adjust_velocity_using_yaw(local_delta, drone_->getOrientation().z());
        Eigen::Vector3d target_pos = drone_->getLocalPosition() + world_delta;

        target_yaw_ += rate_yaw * 0.1f;
        if (target_yaw_ > M_PI) target_yaw_ -= 2.0 * M_PI;
        if (target_yaw_ < -M_PI) target_yaw_ += 2.0 * M_PI;

        float speed = std::abs(vy);
        if (speed < 0.1f) speed = 0.1f;

        move_local_constant_step(drone_, target_pos, speed, 0.1f, target_yaw_);

        return "";
    }

private:
    std::shared_ptr<Drone> drone_;
    bool align_center_;
    bool align_yaw_;
    float tolerance_y_;
    float tolerance_yaw_;
    PidController pid_y_;
    PidController pid_yaw_;
    float target_yaw_;
};
