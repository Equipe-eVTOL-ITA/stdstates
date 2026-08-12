#pragma once

// PrecisionAlignState — centraliza o drone sobre um alvo em coordenadas de
// mundo, descendo à medida que se alinha.
//
// Portado de `precision_align_state.hpp` da fase 1 da CBR 2025, com a visão
// removida. Lá o estado falava direto com o `VisionNode`; aqui ele lê o alvo da
// blackboard, o que o torna reutilizável para qualquer alvo — base, ArUco,
// mangueira — e testável sem câmera.
//
// Contrato com a blackboard:
//
//   entrada  "align_target"      Eigen::Vector3d  posição do alvo em mundo (FRD)
//            "align_target_age"  float            segundos desde a última
//                                                 atualização de "align_target"
//            "align_tolerance"           float
//            "max_horizontal_velocity"   float
//            "align_descent_velocity"    float
//            "detection_timeout"         float
//            "pid_pos_kp" / "_ki" / "_kd" float
//
// A MISSÃO é responsável por manter "align_target" e "align_target_age"
// atualizados a cada ciclo. O estado não sabe de onde vem o alvo.
//
// Outcomes: ""                   (alinhando)
//           "PRECISELY ALIGNED"  (dentro da tolerância por 10 ciclos seguidos)
//           "LOST TARGET"        (alvo velho demais)
//           "ERROR"              (parâmetro ausente)

#include <cmath>
#include <memory>
#include <string>

#include <Eigen/Eigen>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/PidController.hpp"

#include "stdstates/blackboard_params.hpp"

class PrecisionAlignState : public fsm::State
{
public:
  PrecisionAlignState()
  : fsm::State(),
    pid_x_(0.0f, 0.0f, 0.0f, 0.0f, stdstates::kPidSampleTime),
    pid_y_(0.0f, 0.0f, 0.0f, 0.0f, stdstates::kPidSampleTime) {}

  void on_enter(fsm::Blackboard & blackboard) override
  {
    // Todo estado reentrante TEM de reinicializar o seu contador aqui. Em 2025
    // o `over_base` do ReturnHome era inicializado só na declaração do membro,
    // então a segunda entrada no estado começava com o valor da primeira.
    ok_ = false;
    aligned_counter_ = 0;
    have_target_ = false;
    print_counter_ = 0;

    auto drone_ptr = blackboard.get<std::shared_ptr<Drone>>("drone");
    if (drone_ptr == nullptr) return;
    drone_ = *drone_ptr;
    if (drone_ == nullptr) return;

    drone_->log("");
    drone_->log("STATE: PRECISION ALIGN");

    if (!stdstates::require(blackboard, drone_, "align_tolerance", tolerance_)) return;
    if (!stdstates::require(blackboard, drone_, "max_horizontal_velocity", max_velocity_)) return;
    if (!stdstates::require(blackboard, drone_, "align_descent_velocity", descent_velocity_)) return;
    if (!stdstates::require(blackboard, drone_, "detection_timeout", detection_timeout_)) return;

    float kp = 0.0f, ki = 0.0f, kd = 0.0f;
    if (!stdstates::require(blackboard, drone_, "pid_pos_kp", kp)) return;
    if (!stdstates::require(blackboard, drone_, "pid_pos_ki", ki)) return;
    if (!stdstates::require(blackboard, drone_, "pid_pos_kd", kd)) return;

    // Uso idiomático do PidController: setpoint é o ALVO e compute() recebe a
    // MEDIDA, de modo que o erro interno (setpoint - medida) é o deslocamento
    // que falta. O código de 2025 fazia `compute(setpoint - diff.x())` com
    // setpoint = 0, uma dupla negação que dá o mesmo resultado apenas enquanto
    // o setpoint for zero — e vira sinal trocado silencioso se alguém mexer.
    //
    // O sample_time explícito é obrigatório: ver stdstates::kPidSampleTime.
    pid_x_ = PidController(kp, ki, kd, 0.0f, stdstates::kPidSampleTime);
    pid_y_ = PidController(kp, ki, kd, 0.0f, stdstates::kPidSampleTime);
    pid_x_.reset();
    pid_y_.reset();

    ok_ = true;
  }

  std::string act(fsm::Blackboard & blackboard) override
  {
    if (!ok_) return "ERROR";
    print_counter_++;

    const float age = stdstates::optional<float>(blackboard, "align_target_age", 0.0f);
    if (age > detection_timeout_) {
      drone_->log(
        "Alvo perdido: " + std::to_string(age) + "s sem deteccao (limite " +
        std::to_string(detection_timeout_) + "s).");
      drone_->setLocalVelocity(0.0, 0.0, 0.0, 0.0);
      return "LOST TARGET";
    }

    auto * target_ptr = blackboard.get<Eigen::Vector3d>("align_target");
    if (target_ptr != nullptr) {
      target_ = *target_ptr;
      have_target_ = true;
    }

    // CORREÇÃO DE 2025. Lá, quando não havia detecção nos três primeiros
    // ciclos, o código caía direto no cálculo de `diff` usando um
    // `Eigen::Vector3d approx_base` NUNCA INICIALIZADO — Eigen não zera por
    // padrão. O resultado era um comando de velocidade a partir de lixo de
    // pilha, no exato momento em que o drone está descendo sobre uma base.
    // Aqui, sem alvo não há comando: paira e espera.
    if (!have_target_) {
      drone_->setLocalVelocity(0.0, 0.0, 0.0, 0.0);
      return "";
    }

    const Eigen::Vector3d pos = drone_->getLocalPosition();
    const Eigen::Vector2d diff = target_.head<2>() - pos.head<2>();
    const float horizontal_distance = static_cast<float>(diff.norm());

    if (horizontal_distance < tolerance_) {
      aligned_counter_++;
    } else {
      aligned_counter_ = 0;
    }

    // Dez ciclos SEGUIDOS dentro da tolerância, não um. A 20 Hz isso é meio
    // segundo de estabilidade, e é o que separa "passou pelo centro" de "está
    // parado no centro" — a estimativa de visão oscila, e um único ciclo bom
    // acontece por acaso durante a oscilação.
    if (aligned_counter_ > kAlignedCyclesRequired) {
      drone_->setLocalVelocity(0.0, 0.0, 0.0, 0.0);
      drone_->log("Alinhado a " + std::to_string(horizontal_distance) + " m do alvo.");
      return "PRECISELY ALIGNED";
    }

    float x_rate = pid_x_.compute(-static_cast<float>(diff.x()));
    float y_rate = pid_y_.compute(-static_cast<float>(diff.y()));

    Eigen::Vector2d rate(x_rate, y_rate);
    if (rate.norm() > max_velocity_) {
      rate = rate.normalized() * max_velocity_;
    }

    // Só desce quando já está razoavelmente centrado. Descer desalinhado
    // aproxima o alvo da borda do campo de visão e costuma terminar em perda de
    // detecção justamente na fase crítica.
    const float z_rate =
      horizontal_distance < 4.0f * tolerance_ ? descent_velocity_ : 0.0f;

    if (print_counter_ % 10 == 0) {
      drone_->log(
        "Erro horizontal: " + std::to_string(horizontal_distance) +
        " m | descida: " + std::to_string(z_rate) + " m/s");
    }

    drone_->setLocalVelocity(rate.x(), rate.y(), z_rate, 0.0);
    return "";
  }

private:
  static constexpr int kAlignedCyclesRequired = 10;

  std::shared_ptr<Drone> drone_;
  PidController pid_x_, pid_y_;

  Eigen::Vector3d target_ = Eigen::Vector3d::Zero();
  bool have_target_ = false;

  float tolerance_ = 0.0f;
  float max_velocity_ = 0.0f;
  float descent_velocity_ = 0.0f;
  float detection_timeout_ = 0.0f;

  int aligned_counter_ = 0;
  int print_counter_ = 0;
  bool ok_ = false;
};
