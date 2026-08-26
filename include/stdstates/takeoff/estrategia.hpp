#pragma once

// Estrategia — a interface de uma abordagem de decolagem.
//
// Espelha stdstates/landing/estrategia.hpp, e pela mesma razao: a abordagem
// vira um valor (`takeoff_mode` no YAML) em vez de estar escrita no .cpp da
// missao.
//
// O vocabulario de saida e FIXO: "" / "TAKEOFF COMPLETED" / "ERROR". A
// fsm::FSM lanca excecao em outcome fora das transicoes do estado, e uma troca
// de linha no YAML nao pode derrubar a missao em voo. "TAKEOFF COMPLETED" e o
// outcome que todas as missoes ja esperam -- mudar essa string quebraria as
// transicoes de todas elas.

#include <memory>
#include <string>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

namespace stdstates::takeoff
{

inline constexpr const char * kSeguir = "";
inline constexpr const char * kDecolou = "TAKEOFF COMPLETED";
inline constexpr const char * kErro = "ERROR";

class Estrategia
{
public:
  virtual ~Estrategia() = default;

  /// Roda no on_enter, DEPOIS de armar e de reancorar o referencial. false =
  /// nao da para decolar assim, e o estado devolve "ERROR".
  virtual bool preparar(fsm::Blackboard & blackboard, const std::shared_ptr<Drone> & drone) = 0;

  /// Um tick da FSM. `t` = segundos fracionarios desde o preparar().
  virtual std::string passo(const std::shared_ptr<Drone> & drone, double t) = 0;

  /// Chamado ao sair do estado, por qualquer caminho.
  virtual void encerrar(const std::shared_ptr<Drone> & drone) {(void)drone;}

  /// O nome pelo qual esta estrategia e escolhida no YAML.
  virtual const char * nome() const = 0;
};

}  // namespace stdstates::takeoff
