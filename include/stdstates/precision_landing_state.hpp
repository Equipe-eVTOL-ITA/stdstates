#pragma once

// PrecisionLandingState — pouso com descida exponencial e tempo derivado.
//
// Portado de `landing_state.hpp` da fase 1 da CBR 2025.
//
// A ideia boa de 2025, mantida: a velocidade de descida decai de `v_max` a
// `v_min` com constante de tempo CALCULADA a partir das alturas e velocidades
// configuradas, e o timeout também. Nada de número mágico — mudar a altura de
// alinhamento no YAML reajusta o pouso inteiro sozinho.
//
//     v(t) = v_max · e^(−t/τ),  limitado a [v_min, v_max]
//     τ derivado de (v_max − v_min) / (altura_de_alinhamento − altura_da_base)
//
// Contrato com a blackboard:
//
//   entrada  "landing_velocity_max"  float   (positivo, m/s, descida)
//            "landing_velocity_min"  float
//            "align_height"          float   (FRD, NEGATIVO — altura de onde desce)
//            "max_base_height"       float   (FRD, NEGATIVO — topo mais alto possível)
//
// Outcomes: ""        (descendo)
//           "LANDED"  (tempo de pouso esgotado)
//           "ERROR"   (parâmetro ausente ou incoerente)
//
// O que NÃO vem para cá: em 2025 o `on_exit` deste estado registrava a base na
// lista, contava quantas faltavam e escrevia "finished_bases" na blackboard.
// Isso é contabilidade da fase 1 — um estado genérico de pouso não pode saber o
// que é uma "base". A missão faz isso no seu próprio estado seguinte.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <Eigen/Eigen>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

#include "stdstates/blackboard_params.hpp"

class PrecisionLandingState : public fsm::State
{
public:
  PrecisionLandingState()
  : fsm::State() {}

  void on_enter(fsm::Blackboard & blackboard) override
  {
    ok_ = false;

    auto drone_ptr = blackboard.get<std::shared_ptr<Drone>>("drone");
    if (drone_ptr == nullptr) return;
    drone_ = *drone_ptr;
    if (drone_ == nullptr) return;

    drone_->log("");
    drone_->log("STATE: PRECISION LANDING");

    if (!stdstates::require(blackboard, drone_, "landing_velocity_max", v_max_)) return;
    if (!stdstates::require(blackboard, drone_, "landing_velocity_min", v_min_)) return;

    float align_height = 0.0f, max_base_height = 0.0f;
    if (!stdstates::require(blackboard, drone_, "align_height", align_height)) return;
    if (!stdstates::require(blackboard, drone_, "max_base_height", max_base_height)) return;

    // As alturas vêm em FRD (negativas para cima do solo). A conta é feita em
    // distâncias positivas.
    const float queda = -align_height - (-max_base_height);

    // GUARDAS QUE 2025 NÃO TINHA. Sem elas, um YAML com align_height igual a
    // max_base_height divide por zero e produz tau = inf; um v_min de 0
    // produz log(inf) no cálculo do tempo. Nos dois casos o drone desce com
    // velocidade NaN, que o PX4 rejeita silenciosamente — ele simplesmente
    // fica parado no ar até alguém desistir e desarmar.
    if (v_min_ <= 0.0f || v_max_ <= v_min_) {
      drone_->log(
        "ERRO: velocidades de pouso incoerentes (max=" + std::to_string(v_max_) +
        ", min=" + std::to_string(v_min_) + "). Exige-se 0 < min < max.");
      return;
    }
    if (queda <= 0.0f) {
      drone_->log(
        "ERRO: align_height deve estar ACIMA de max_base_height (queda=" +
        std::to_string(queda) + " m). Ambos sao negativos em FRD.");
      return;
    }

    decay_rate_ = (v_max_ - v_min_) / queda;                       // 1/s
    const double t_ate_v_min = std::log(v_max_ / v_min_) / decay_rate_;
    const double t_total = t_ate_v_min + (-max_base_height) / v_min_;
    timeout_s_ = t_total + kMargemSegurancaS;

    start_time_ = std::chrono::steady_clock::now();

    drone_->log(
      "Descida: " + std::to_string(queda) + " m, tau=" +
      std::to_string(1.0f / decay_rate_) + " s");
    drone_->log(
      "Tempo previsto de pouso: " + std::to_string(t_total) +
      " s (timeout " + std::to_string(timeout_s_) + " s)");

    ok_ = true;
  }

  std::string act(fsm::Blackboard & blackboard) override
  {
    (void)blackboard;
    if (!ok_) return "ERROR";

    // Em SEGUNDOS FRACIONÁRIOS. O código de 2025 usava
    // `duration_cast<std::chrono::seconds>`, que TRUNCA para inteiro: a
    // velocidade só mudava uma vez por segundo, o que a 20 Hz significa vinte
    // comandos idênticos seguidos de um degrau. A descida "exponencial" era na
    // verdade uma escada, e o solavanco a cada degrau é exatamente o tipo de
    // coisa que se confunde com problema de sintonia do controlador.
    const std::chrono::duration<double> elapsed =
      std::chrono::steady_clock::now() - start_time_;
    const double t = elapsed.count();

    if (t > timeout_s_) {
      drone_->setLocalVelocity(0.0, 0.0, 0.0, 0.0);
      drone_->log("Pouso concluido apos " + std::to_string(t) + " s.");
      return "LANDED";
    }

    const float velocity = std::clamp(
      static_cast<float>(v_max_ * std::exp(-decay_rate_ * t)), v_min_, v_max_);

    // z positivo é para BAIXO em FRD: velocidade positiva desce.
    drone_->setLocalVelocity(0.0, 0.0, velocity, 0.0);
    return "";
  }

private:
  /// Folga sobre o tempo calculado, para cobrir a diferença entre a velocidade
  /// comandada e a efetivamente atingida pelo controlador.
  static constexpr double kMargemSegurancaS = 5.0;

  std::shared_ptr<Drone> drone_;

  float v_max_ = 0.0f;
  float v_min_ = 0.0f;
  float decay_rate_ = 0.0f;
  double timeout_s_ = 0.0;
  std::chrono::steady_clock::time_point start_time_;
  bool ok_ = false;
};
