#pragma once

// Px4Land — deixa o pouso com o proprio PX4 (VEHICLE_CMD_NAV_LAND).
//
// POR QUE ESTA OPCAO EXISTE
//
// As outras duas descem por setpoint de velocidade em offboard: o perfil e
// nosso, e o PX4 so obedece. Isso da controle -- e da a responsabilidade.
// O modo LAND do PX4 usa o detector de solo do proprio firmware, que enxerga
// coisas que a missao nao enxerga: corrente de ar, empuxo caindo ao encostar,
// o solo mais alto do que se esperava. Em dia de vento, ou sobre um piso cuja
// altura nao se conhece, ele pousa melhor do que qualquer perfil aberto.
//
// O QUE MUDA, E QUE PRECISA SER SABIDO ANTES DE ESCOLHER
//
//   1. LAND TIRA O DRONE DO MODO OFFBOARD. Uma missao que pousa e redecola --
//      a fase 1 pousa em cada base -- volta ao TakeoffState, que ja rearma e
//      reentra em offboard quando encontra o drone desarmado. Funciona, mas a
//      redecolagem passa a custar os ~2 s do toOffboardSync.
//
//   2. O DRONE DESARMA. Isso e o pouso "de verdade", e nao um pairar rente ao
//      chao. Se a missao contava com as helices girando logo apos, escolher
//      este modo muda o comportamento dela.
//
//   3. O TEMPO NAO E CALCULADO AQUI. Quem decide a velocidade de descida e o
//      PX4, pelos seus proprios parametros (MPC_LAND_SPEED e afins) -- os
//      `landing_velocity_*` do YAML sao IGNORADOS neste modo. Nao ha como
//      honra-los sem deixar de usar o modo LAND.
//
//   opcional  "disarm_grace"    float  s antes de forcar o desarme (padrao 3)
//   opcional  "disarm_timeout"  float  s ate desistir (padrao 20)
//
// Sobre o timeout: aqui ele vira "ERROR", e nao um outcome proprio. O
// LandAndDisarmState existe justamente para quem precisa distinguir "nao
// desarmou" de "falhou" -- e continua existindo, com o outcome "TIMEOUT". A
// razao de nao inventar outcome aqui esta no estrategia.hpp: uma troca de YAML
// nao pode derrubar a FSM com um outcome que as transicoes da missao nao
// conhecem.

#include <memory>
#include <string>

#include "stdstates/blackboard_params.hpp"
#include "stdstates/landing/estrategia.hpp"

namespace stdstates::landing
{

class Px4Land : public Estrategia
{
public:
  const char * nome() const override {return "px4";}

  bool preparar(fsm::Blackboard & blackboard, const std::shared_ptr<Drone> & drone) override
  {
    grace_s_ = stdstates::optional<float>(blackboard, "disarm_grace", 3.0f);
    timeout_s_ = stdstates::optional<float>(blackboard, "disarm_timeout", 20.0f);
    pediu_desarme_ = false;

    // O modo LAND desce e desarma sozinho ao detectar o solo. Pedir o desarme
    // antes disso pode ser recusado -- dai a carencia.
    drone->log(
      "Pouso PX4 (modo LAND); carencia " + std::to_string(grace_s_) +
      " s, timeout " + std::to_string(timeout_s_) + " s");
    drone->land();
    return true;
  }

  std::string passo(const std::shared_ptr<Drone> & drone, double t) override
  {
    if (drone->getArmingState() == DronePX4::ARMING_STATE::DISARMED) {
      drone->log("Pousado e desarmado apos " + std::to_string(t) + " s.");
      return kPousado;
    }

    if (t > timeout_s_) {
      drone->log(
        "ERRO: o modo LAND nao desarmou em " + std::to_string(timeout_s_) +
        " s. O drone pode continuar armado.");
      return kErro;
    }

    // Passada a carencia, insiste. `disarm()` e o comando de um tiro;
    // `disarmSync()` seria o laco bloqueante que congela o executor inteiro
    // dentro do callback do timer da FSM.
    if (t > grace_s_) {
      if (!pediu_desarme_) {
        drone->log("Nao desarmou sozinho; forcando.");
        pediu_desarme_ = true;
      }
      drone->disarm();
    }

    return kSeguir;
  }

  void encerrar(const std::shared_ptr<Drone> & drone) override
  {
    // NAO manda setLocalVelocity aqui, ao contrario das outras estrategias.
    //
    // Um setpoint de velocidade publicado depois do LAND reintroduz a intencao
    // de offboard num drone que o PX4 ja esta pousando ou ja pousou. No melhor
    // caso e ignorado; no pior, disputa o controle com o modo LAND a centimetros
    // do chao.
    (void)drone;
  }

private:
  float grace_s_ = 3.0f;
  float timeout_s_ = 20.0f;
  bool pediu_desarme_ = false;
};

}  // namespace stdstates::landing
