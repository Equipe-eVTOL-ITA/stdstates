#pragma once

// Exponencial — o pouso padrao do time.
//
//     v(t) = v_max · e^(−t/τ),  limitado a [v_min, v_max]
//     τ derivado de (v_max − v_min) / (altura_de_partida − altura_da_base)
//
// Nada de numero magico: mudar a altura no YAML reajusta o perfil e o timeout.
// Matematica movida do PrecisionLandingState sem mudanca de comportamento.
//
//   entrada  landing_velocity_max/min, max_base_height
//   opcional landing_timeout (folga em s; padrao 5)
//
// A altura de PARTIDA nao e parametro: e medida ao entrar no estado.

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "stdstates/blackboard_params.hpp"
#include "stdstates/landing/estrategia.hpp"

namespace stdstates::landing
{

class Exponencial : public Estrategia
{
public:
  const char * nome() const override {return "exponencial";}

  bool preparar(fsm::Blackboard & blackboard, const std::shared_ptr<Drone> & drone) override
  {
    if (!stdstates::require(blackboard, drone, "landing_velocity_max", v_max_)) {return false;}
    if (!stdstates::require(blackboard, drone, "landing_velocity_min", v_min_)) {return false;}

    float max_base_height = 0.0f;
    if (!stdstates::require(blackboard, drone, "max_base_height", max_base_height)) {return false;}

    // Sem estas guardas, uma queda nula da tau = inf e a descida vira NaN, que
    // o PX4 rejeita em silencio: o drone fica parado no ar.
    if (v_min_ <= 0.0f || v_max_ <= v_min_) {
      drone->log(
        "ERRO: velocidades de pouso incoerentes (max=" + std::to_string(v_max_) +
        ", min=" + std::to_string(v_min_) + "). Exige-se 0 < min < max.");
      return false;
    }

    // Altura de partida SEMPRE medida agora, e nunca lida do YAML: na fase 3 o
    // drone pousa de onde o gesto o deixou, que e qualquer altura.
    const float altura_atual = -static_cast<float>(drone->getLocalPosition().z());
    const float altura_alvo = alturaDaBase(max_base_height, drone);

    const float margem = stdstates::optional<float>(
      blackboard, "landing_timeout", static_cast<float>(kMargemSegurancaS));

    const float queda = altura_atual - altura_alvo;

    if (queda <= 0.0f) {
      // Nao e erro: por gesto da para descer ate quase encostar antes de
      // mandar pousar.
      decay_rate_ = 1.0f;                     // irrelevante: ja saturado em v_min
      timeout_s_ = altura_atual / v_min_ + margem;
      drone->log(
        "Ja abaixo da altura de base (" + std::to_string(altura_atual) +
        " m); descendo direto a " + std::to_string(v_min_) + " m/s.");
    } else {
      decay_rate_ = (v_max_ - v_min_) / queda;                        // 1/s
      const double t_ate_v_min = std::log(v_max_ / v_min_) / decay_rate_;
      const double t_total = t_ate_v_min + altura_alvo / v_min_;
      timeout_s_ = t_total + margem;
    }

    drone->log(
      "Pouso EXPONENCIAL de " + std::to_string(altura_atual) + " m ate " +
      std::to_string(altura_alvo) + " m; tau=" + std::to_string(1.0f / decay_rate_) +
      " s, timeout " + std::to_string(timeout_s_) + " s");
    return true;
  }

  std::string passo(const std::shared_ptr<Drone> & drone, double t) override
  {
    if (t > timeout_s_) {
      drone->setLocalVelocity(0.0, 0.0, 0.0, 0.0);
      drone->log("Pouso concluido apos " + std::to_string(t) + " s.");
      return kPousado;
    }

    const float velocity = std::clamp(
      static_cast<float>(v_max_ * std::exp(-decay_rate_ * t)), v_min_, v_max_);

    // z positivo e para BAIXO em FRD: velocidade positiva desce.
    drone->setLocalVelocity(0.0, 0.0, velocity, 0.0);
    return kSeguir;
  }

private:
  float v_max_ = 0.0f;
  float v_min_ = 0.0f;
  float decay_rate_ = 0.0f;
  double timeout_s_ = 0.0;
};

}  // namespace stdstates::landing
