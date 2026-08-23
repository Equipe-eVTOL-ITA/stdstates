#pragma once

// Px4Land — deixa o pouso com o firmware (VEHICLE_CMD_NAV_LAND).
//
// Usa o detector de solo do PX4, que enxerga o que a missao nao enxerga:
// vento, empuxo caindo ao encostar, solo mais alto do que se esperava.
//
// O QUE MUDA, e precisa ser sabido antes de escolher:
//   1. TIRA o drone de offboard -- a redecolagem passa a custar o
//      toOffboardSync (~2 s), que o TakeoffState ja faz sozinho.
//   2. DESARMA. Nao e um pairar rente ao chao.
//   3. IGNORA os landing_velocity_*: quem decide a descida e o MPC_LAND_SPEED.
//
//   opcional  disarm_grace (3 s), disarm_timeout (20 s)
//
// Timeout vira "ERROR", e nao outcome proprio -- ver estrategia.hpp. Quem
// precisa distinguir "nao desarmou" de "falhou" usa o LandAndDisarmState.

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

    // LAND desarma sozinho ao detectar o solo; pedir antes pode ser recusado.
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

    // disarm() e de um tiro; disarmSync() bloquearia o executor inteiro.
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
    // Sem setLocalVelocity aqui: um setpoint depois do LAND disputaria o
    // controle com o firmware a centimetros do chao.
    (void)drone;
  }

private:
  float grace_s_ = 3.0f;
  float timeout_s_ = 20.0f;
  bool pediu_desarme_ = false;
};

}  // namespace stdstates::landing
