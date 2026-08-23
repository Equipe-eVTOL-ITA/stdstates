#pragma once

// stdstates::motion — a politica de movimento vinda da blackboard.
//
// Um helper so, para que nenhum estado leia `motion_policy` por conta propria:
// um deles esqueceria, e continuaria voando holonomico enquanto o YAML diz
// axial -- em silencio.

#include <memory>
#include <string>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "drone/motion_policy.hpp"

#include "stdstates/blackboard_params.hpp"

namespace stdstates
{

/// Cria a politica declarada em `motion_policy`. nullptr = nome invalido,
/// e nesse caso o estado deve devolver "ERROR".
inline std::unique_ptr<drone::MotionPolicy> criarPolitica(
  fsm::Blackboard & blackboard, const std::shared_ptr<Drone> & drone)
{
  const std::string nome = stdstates::optional<std::string>(
    blackboard, "motion_policy", drone::kPoliticaPadrao);

  auto politica = drone::criarPolitica(nome);
  if (politica == nullptr) {
    if (drone != nullptr) {
      drone->log("ERRO: motion_policy desconhecida: '" + nome + "'.");
      drone->log("      Politicas aceitas: " + drone::politicasDisponiveis());
    }
    return politica;
  }

  // Anuncia a politica em uso. Uma escolha que muda como o drone voa tem de
  // aparecer no log -- foi assim que se descobriu um YAML editado no arquivo
  // errado (flight.yaml em vez de simulation.yaml).
  if (drone != nullptr) {
    drone->log(std::string("Movimento: ") + politica->nome());
  }
  return politica;
}

/// Os limites de deslocamento que a missao declara.
///
/// `passo` sai de `max_horizontal_velocity`, que NAO e uma velocidade e sim a
/// distancia a que o setpoint e posto a frente. O nome ficou de 2025.
inline drone::Limites limitesDaBlackboard(
  fsm::Blackboard & blackboard, float tolerancia_posicao)
{
  drone::Limites lim;
  lim.passo = stdstates::optional<float>(blackboard, "max_horizontal_velocity", 1.0f);
  lim.posicao = tolerancia_posicao;
  lim.yaw = stdstates::optional<float>(blackboard, "motion_yaw_tolerance", 0.05f);
  return lim;
}

}  // namespace stdstates
