#pragma once

#include <Eigen/Eigen>
#include <cstdint>
#include <memory>
#include <string>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/movement.hpp"

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
// >>> CONTRATO px4.reancoragem-do-home
// setHomePosition() REANCORA o referencial do mundo na posicao e no yaw atuais.
// Use TakeoffState(true) SO na decolagem inicial da missao.
//
// Numa redecolagem no meio da missao ela e destrutiva: a origem do mundo pula
// para onde o drone estiver, e tudo o que estava guardado em coordenadas de
// mundo -- bases ja visitadas, a grade de varredura, a posicao de casa --
// passa a se referir a um referencial que nao existe mais.
//
// Nao ha erro. O drone decola, olha para baixo, ve a base em que acabou de
// pousar, nao a reconhece, e pousa nela de novo. E de novo.
//
// Medido em SITL: o NED cru do PX4 ficou em (3.417, -0.159) o ciclo inteiro --
// o drone nunca saiu do lugar -- enquanto o FRD visto pela missao saltava de
// (-0.16, -3.03) para (0.00, 0.03) a cada redecolagem.
// <<< CONTRATO

class TakeoffState : public fsm::State {
public:
    /// @param set_home  Se true, reancora o referencial FRD na posicao atual
    ///                   ao entrar no estado. Verdadeiro para a decolagem
    ///                   INICIAL da missao; FALSO para qualquer redecolagem.
    ///                   Ver o comentario extenso no on_enter.
    explicit TakeoffState(bool set_home = true)
    : fsm::State(), set_home_(set_home) {}

    void on_enter(fsm::Blackboard &blackboard) override {
        drone_ = *blackboard.get<std::shared_ptr<Drone>>("drone");
        if (drone_ == nullptr) return;

        drone_->log("");
        drone_->log("STATE: TAKEOFF");

        max_velocity_ = *blackboard.get<float>("max_vertical_velocity");
        position_tolerance_ = *blackboard.get<float>("position_tolerance");
        float takeoff_height = *blackboard.get<float>("takeoff_height");

        if (drone_->getArmingState() != DronePX4::ARMING_STATE::ARMED) {
            drone_->toOffboardSync();
            drone_->armSync();
        }

        // setHomePosition REANCORA O REFERENCIAL DO MUNDO na posicao e no yaw
        // atuais do drone: ela grava ned_home_position_ = posicao NED de agora,
        // e todo getLocalPosition() posterior passa a ser medido a partir dali.
        //
        // Isso e o que se quer na decolagem INICIAL -- e preciso chama-la depois
        // de armar, para que o EKF do PX4 ja tenha convergido para o heading
        // verdadeiro; chamar antes deixa initial_yaw_ em 0 enquanto o heading
        // real e outro, e o primeiro setpoint de posicao produz uma guinada
        // inesperada.
        //
        // Mas e DESTRUTIVO em qualquer redecolagem no meio da missao. Uma FSM
        // que pousa e redecola (varrer uma arena pousando em varias bases, por
        // exemplo) volta a este estado, e a origem do mundo pula para a base
        // recem-visitada. Tudo o que estava guardado em coordenadas de mundo --
        // a lista de bases ja visitadas, a grade de varredura, a posicao de casa
        // -- passa a se referir a um referencial que nao existe mais.
        //
        // Nao ha erro. O drone decola, olha para baixo, ve a base em que acabou
        // de pousar, nao a reconhece porque as coordenadas guardadas viraram
        // outra coisa, e pousa nela de novo. E de novo.
        //
        // Medido em SITL: o NED cru do PX4 ficou em (3.417, -0.159) durante todo
        // o ciclo -- o drone nunca saiu do lugar -- enquanto o FRD visto pela
        // missao saltava de (-0.16, -3.03) para (0.00, 0.03) a cada redecolagem.
        if (set_home_) {
            drone_->setHomePosition(Eigen::Vector3d(0, 0, 0));
        }

        pos_ = drone_->getLocalPosition();
        // Zero logo apos setHomePosition; na redecolagem e o yaw acumulado
        // desde a decolagem inicial, que e justamente o que se quer preservar.
        initial_yaw_ = drone_->getOrientation()[2];
        goal_ = Eigen::Vector3d(pos_[0], pos_[1], takeoff_height);

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

        move_local_constant_step(drone_, goal_, max_velocity_, position_tolerance_, initial_yaw_);

        return "";
    }

private:
    bool set_home_;
    std::shared_ptr<Drone> drone_;
    Eigen::Vector3d pos_, goal_;
    float max_velocity_;
    float initial_yaw_;
    float position_tolerance_;
    int print_counter_;
};