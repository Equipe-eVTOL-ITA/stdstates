#pragma once

// Px4Takeoff — deixa a decolagem com o firmware (VEHICLE_CMD_NAV_TAKEOFF).
//
// O PX4 entra em AUTO_TAKEOFF, sobe sozinho e termina em AUTO_LOITER. Usa a
// rampa e a protecao dele, que conhecem o veiculo melhor do que um passo
// constante nosso.
//
// O QUE MUDA, e precisa ser sabido antes de escolher:
//
//   1. A ALTURA E A DO FIRMWARE, e nao a da missao. Sem posicao global esta
//      camada nao tem como passar uma altitude AMSL, entao o PX4 sobe ate o
//      proprio MIS_TAKEOFF_ALT -- o `takeoff_height` do YAML e ignorado. E o
//      mesmo tipo de troca do modo LAND, que ignora as landing_velocity_*.
//
//   2. TIRA O DRONE DE OFFBOARD. O `encerrar()` reentra em offboard antes de
//      devolver o controle a missao, porque o estado seguinte vai mandar
//      setpoint e seria ignorado em AUTO_LOITER. Isso custa os ~2 s do
//      toOffboardSync.
//
//   opcional  "takeoff_timeout"  float  s ate desistir (padrao 30)
//
// A conclusao e detectada pelo MODO: o drone entra em AUTO_TAKEOFF e sai dele
// quando termina. Esperar pela altura nao serviria -- a altura de destino e do
// firmware, e a missao nao a conhece.

#include <memory>
#include <string>

#include "stdstates/blackboard_params.hpp"
#include "stdstates/takeoff/estrategia.hpp"

namespace stdstates::takeoff
{

class Px4Takeoff : public Estrategia
{
public:
  const char * nome() const override {return "px4";}

  bool preparar(fsm::Blackboard & blackboard, const std::shared_ptr<Drone> & drone) override
  {
    timeout_s_ = stdstates::optional<float>(blackboard, "takeoff_timeout", 30.0f);
    entrou_ = false;

    drone->log(
      "Decolagem PX4 (AUTO_TAKEOFF ate o MIS_TAKEOFF_ALT do firmware; "
      "o takeoff_height do YAML NAO se aplica)");
    drone->takeoff();
    return true;
  }

  std::string passo(const std::shared_ptr<Drone> & drone, double t) override
  {
    const auto modo = drone->getFlightMode();

    if (modo == DronePX4::FLIGHT_MODE::AUTO_TAKEOFF) {
      entrou_ = true;
      return kSeguir;
    }

    // So conta como concluida depois de ter ENTRADO em AUTO_TAKEOFF: nos
    // primeiros ticks o modo ainda e o anterior, e sair dele sem ter entrado
    // significa apenas que o comando ainda nao foi processado.
    if (entrou_) {
      const Eigen::Vector3d pos = drone->getLocalPosition();
      drone->log("Takeoff completed at altitude " + std::to_string(pos[2]));
      return kDecolou;
    }

    if (t > timeout_s_) {
      drone->log(
        "ERRO: o PX4 nao entrou em AUTO_TAKEOFF em " + std::to_string(timeout_s_) +
        " s. O drone esta armado? O modo atual e " + std::to_string(static_cast<int>(modo)) + ".");
      return kErro;
    }

    return kSeguir;
  }

  void encerrar(const std::shared_ptr<Drone> & drone) override
  {
    if (drone == nullptr) {return;}
    // Sem isto o estado seguinte manda setpoint em AUTO_LOITER, e o PX4 os
    // ignora em silencio -- o drone fica parado no ar e a missao "trava" sem
    // erro nenhum.
    drone->log("Reentrando em offboard apos a decolagem do PX4.");
    drone->toOffboardSync();
  }

private:
  float timeout_s_ = 30.0f;
  bool entrou_ = false;
};

}  // namespace stdstates::takeoff
