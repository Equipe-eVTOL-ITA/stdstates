#pragma once

// stdstates::motion — a politica de movimento vinda da blackboard.
//
// Um helper so, para que todo estado que se desloca crie a politica do MESMO
// jeito: lendo `motion_policy` do YAML, caindo no padrao quando a chave nao
// existe, e recusando ALTO um nome que nao existe.
//
// POR QUE NAO CADA ESTADO LER A CHAVE POR CONTA
//
// Porque um deles esqueceria. E o modo de esquecer e silencioso: o estado
// continuaria voando holonomico enquanto o YAML diz `axial`, e a missao teria
// metade dos deslocamentos obedecendo a regra e metade nao. No dia em que a
// regra for "so pode girar e ir para frente", "quase todos os estados" nao
// serve de nada.

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

  // ANUNCIA A POLITICA EM USO, sempre.
  //
  // O estado de pouso ja dizia "Pouso EXPONENCIAL ..." no log, e foi assim que
  // se descobriu, num voo em que a configuracao pedia outra coisa, que o
  // arquivo carregado nao era o editado -- a missao de simulacao le o
  // simulation.yaml, e a edicao tinha ido para o flight.yaml.
  //
  // O movimento nao dizia nada, e por isso a metade dele do mesmo engano ficou
  // invisivel. Uma escolha que muda como o drone voa tem de aparecer no log:
  // custa uma linha, e e a diferenca entre ver o problema e procura-lo.
  if (drone != nullptr) {
    drone->log(std::string("Movimento: ") + politica->nome());
  }
  return politica;
}

/// Os limites de deslocamento que a missao declara.
///
/// `passo` sai de `max_horizontal_velocity`, que e como os estados sempre o
/// chamaram -- apesar de o valor NAO ser uma velocidade, e sim a distancia a
/// que o setpoint e posto a frente. O nome ficou de 2025 e mudá-lo quebraria
/// todos os YAML de todas as missoes; o que se pode fazer e dizer aqui o que
/// ele e de fato.
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
