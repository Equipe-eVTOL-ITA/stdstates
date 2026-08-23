#pragma once

// SCurve — descida com perfil em S, sem solavanco no inicio nem no fim.
//
// POR QUE UMA ALTERNATIVA A EXPONENCIAL
//
// A exponencial comeca em v_max NO PRIMEIRO TICK: o drone vem pairando a 0 m/s
// e recebe, de uma vez, "desca a 0,5 m/s". O degrau esta na aceleracao, e o
// controlador do PX4 responde com um mergulho seguido de correcao -- que numa
// camera apontada para baixo aparece como a base saindo e voltando ao quadro
// justo quando o alinhamento acabou de convergir.
//
// O perfil em S sai de v_max e chega a v_min com DERIVADA NULA nas duas
// pontas. A funcao e o smoothstep classico:
//
//     s(x) = 3x² − 2x³        s(0)=0, s(1)=1, s'(0)=s'(1)=0
//     v(t) = v_min + (v_max − v_min) · (1 − s(t/T))
//
// O T NAO E ARBITRADO, e sim resolvido para cobrir a queda exatamente.
// Como ∫₀¹ s(x) dx = 1/2, a velocidade media do perfil e (v_max + v_min)/2, e:
//
//     T = queda / ((v_max + v_min) / 2)
//
// Passado T, mantem v_min ate o timeout -- igual a exponencial, e pela mesma
// razao: o ultimo trecho ate encostar tem de ser lento.
//
// Mesmos parametros da exponencial. Trocar entre as duas e trocar
// `landing_mode` no YAML.

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include "stdstates/blackboard_params.hpp"
#include "stdstates/landing/estrategia.hpp"

namespace stdstates::landing
{

class SCurve : public Estrategia
{
public:
  const char * nome() const override {return "s_curve";}

  bool preparar(fsm::Blackboard & blackboard, const std::shared_ptr<Drone> & drone) override
  {
    if (!stdstates::require(blackboard, drone, "landing_velocity_max", v_max_)) {return false;}
    if (!stdstates::require(blackboard, drone, "landing_velocity_min", v_min_)) {return false;}

    float max_base_height = 0.0f;
    if (!stdstates::require(blackboard, drone, "max_base_height", max_base_height)) {return false;}

    if (v_min_ <= 0.0f || v_max_ <= v_min_) {
      drone->log(
        "ERRO: velocidades de pouso incoerentes (max=" + std::to_string(v_max_) +
        ", min=" + std::to_string(v_min_) + "). Exige-se 0 < min < max.");
      return false;
    }

    const float altura_atual = -static_cast<float>(drone->getLocalPosition().z());
    const float altura_alvo = alturaDaBase(max_base_height, drone);
    const float margem = stdstates::optional<float>(
      blackboard, "landing_timeout", static_cast<float>(kMargemSegurancaS));

    const float queda = altura_atual - altura_alvo;

    if (queda <= 0.0f) {
      // Ja na altura do alvo ou abaixo: nao ha rampa a fazer. Mesmo tratamento
      // da exponencial, e pela mesma razao.
      t_rampa_ = 0.0;
      timeout_s_ = altura_atual / v_min_ + margem;
      drone->log(
        "Ja abaixo da altura de base (" + std::to_string(altura_atual) +
        " m); descendo direto a " + std::to_string(v_min_) + " m/s.");
    } else {
      const double v_media = 0.5 * (v_max_ + v_min_);
      t_rampa_ = queda / v_media;
      timeout_s_ = t_rampa_ + altura_alvo / v_min_ + margem;
    }

    drone->log(
      "Pouso S-CURVE de " + std::to_string(altura_atual) + " m ate " +
      std::to_string(altura_alvo) + " m; rampa " + std::to_string(t_rampa_) +
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

    float velocity = v_min_;
    if (t < t_rampa_ && t_rampa_ > 0.0) {
      const double x = t / t_rampa_;
      const double s = x * x * (3.0 - 2.0 * x);          // 3x² − 2x³
      velocity = static_cast<float>(v_min_ + (v_max_ - v_min_) * (1.0 - s));
    }
    velocity = std::clamp(velocity, v_min_, v_max_);

    // z positivo e para BAIXO em FRD: velocidade positiva desce.
    drone->setLocalVelocity(0.0, 0.0, velocity, 0.0);
    return kSeguir;
  }

private:
  float v_max_ = 0.0f;
  float v_min_ = 0.0f;
  double t_rampa_ = 0.0;
  double timeout_s_ = 0.0;
};

}  // namespace stdstates::landing
