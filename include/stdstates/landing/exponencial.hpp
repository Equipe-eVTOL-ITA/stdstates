#pragma once

// Exponencial — a descida exponencial, que e o pouso padrao do time.
//
//     v(t) = v_max · e^(−t/τ),  limitado a [v_min, v_max]
//     τ derivado de (v_max − v_min) / (altura_de_partida − altura_da_base)
//
// A ideia boa de 2025, mantida: nada de numero magico. Mudar a altura no YAML
// reajusta o perfil inteiro e o timeout sozinho.
//
// Este arquivo e a matematica que estava dentro do PrecisionLandingState,
// movida para ca SEM MUDANCA DE COMPORTAMENTO -- de proposito. Uma missao que
// nao declara `landing_mode` continua pousando exatamente como antes, e essa e
// a unica forma de saber que a extracao nao introduziu nada.
//
//   entrada  "landing_velocity_max"  float  (positivo, m/s, descida)
//            "landing_velocity_min"  float
//            "max_base_height"       float  (FRD, negativo -- ver alturaDaBase)
//   opcional "landing_timeout"       float  (folga em s; padrao 5)
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

    // GUARDAS QUE 2025 NAO TINHA. Sem elas, uma queda nula divide por zero e
    // produz tau = inf; um v_min de 0 produz log(inf). Nos dois casos o drone
    // desce com velocidade NaN, que o PX4 rejeita silenciosamente -- ele
    // simplesmente fica parado no ar ate alguem desistir e desarmar.
    if (v_min_ <= 0.0f || v_max_ <= v_min_) {
      drone->log(
        "ERRO: velocidades de pouso incoerentes (max=" + std::to_string(v_max_) +
        ", min=" + std::to_string(v_min_) + "). Exige-se 0 < min < max.");
      return false;
    }

    // A ALTURA DE PARTIDA E SEMPRE A ATUAL, MEDIDA AGORA.
    //
    // A primeira versao lia `align_height` do YAML, porque foi portada da fase
    // 1, onde o alinhamento sempre leva o drone a mesma altitude antes de
    // pousar. La o valor de config e a altura real quase coincidem -- mas nem
    // la sao iguais, porque o alinhamento ja desce um pouco enquanto converge.
    //
    // Em geral nao coincidem nem de longe: na fase 3 o drone pousa de onde o
    // operador o deixou por gesto, que e qualquer altura. Um valor fixo faria o
    // perfil ser calculado para uma queda que nao existe -- curto demais, o
    // drone chega ao solo ainda rapido; longo demais, fica parado no ar
    // esperando um tempo que nao passa.
    const float altura_atual = -static_cast<float>(drone->getLocalPosition().z());
    const float altura_alvo = alturaDaBase(max_base_height, drone);

    const float margem = stdstates::optional<float>(
      blackboard, "landing_timeout", static_cast<float>(kMargemSegurancaS));

    const float queda = altura_atual - altura_alvo;

    if (queda <= 0.0f) {
      // O drone ja esta na altura do alvo ou abaixo dela. Nao e erro: com
      // controle por gesto o operador pode descer ate quase encostar e so
      // entao mandar pousar. Abortar aqui deixaria o drone pairando a 20 cm do
      // chao sem explicacao.
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
