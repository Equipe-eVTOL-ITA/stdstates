#pragma once

// Leitura segura de parâmetros da blackboard.
//
// POR QUE ISTO EXISTE
//
// `fsm::Blackboard::get<T>(chave)` devolve um PONTEIRO CRU e `nullptr` quando a
// chave não existe. O uso idiomático no código do time é
//
//     float tol = *blackboard.get<float>("position_tolerance");
//
// que é um dereference de `nullptr` — segfault — se alguém errar o nome do
// parâmetro no YAML, ou esquecer de declará-lo, ou renomeá-lo numa fase e não
// na outra. Não há aviso do compilador: o tipo está certo, a chave é uma
// string, e o erro só aparece em voo.
//
// Pior: `get<T>` faz `(Value<T>*)` — um cast C SEM CHECAGEM. Gravar `double` e
// ler `float` compila, roda e devolve lixo. O TIPO DA CHAVE É PARTE DO
// CONTRATO. Todo o `stdstates` lê `float`; a missão tem de gravar `float`.
//
// Estes helpers trocam um segfault silencioso por uma mensagem que diz qual
// chave faltou.

#include <memory>
#include <string>
#include <vector>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

namespace stdstates {

/// Lê um parâmetro obrigatório. Devolve false e registra no log qual chave
/// faltou, em vez de derreferenciar nullptr.
///
/// Uso no `on_enter`, que não pode devolver outcome:
///
///     if (!stdstates::require(blackboard, drone_, "position_tolerance", tol_)) {
///         faltou_parametro_ = true;
///         return;
///     }
///
/// e então `act` devolve "ERROR" enquanto `faltou_parametro_` for verdadeiro.
template <typename T>
bool require(fsm::Blackboard & blackboard,
             const std::shared_ptr<Drone> & drone,
             const std::string & key,
             T & out)
{
  T * ptr = blackboard.get<T>(key);
  if (ptr == nullptr) {
    if (drone != nullptr) {
      drone->log("ERRO: parametro ausente na blackboard: '" + key + "'");
    }
    return false;
  }
  out = *ptr;
  return true;
}

/// Lê um parâmetro opcional, com valor padrão quando a chave não existe.
/// Use apenas quando o padrão for genuinamente seguro — não para esconder
/// parâmetro esquecido.
template <typename T>
T optional(fsm::Blackboard & blackboard, const std::string & key, const T & fallback)
{
  T * ptr = blackboard.get<T>(key);
  return ptr != nullptr ? *ptr : fallback;
}

/// Período de amostragem dos PIDs dos estados.
///
/// NÃO use o padrão de 0.05 s do `PidController`. A FSM roda num timer de 50 ms,
/// e `PidController::compute()` devolve **0.0f** — não o último valor — quando
/// chamado antes de `sample_time` ter passado. Com os dois iguais a 0.05 s, o
/// controle fica exatamente no limite: qualquer jitter de escalonamento produz
/// zeros intermitentes na saída, que aparecem como um drone "engasgando" e são
/// facilmente confundidos com PID mal sintonizado.
///
/// 0.04 s dá 20% de folga sobre o período do timer.
inline constexpr float kPidSampleTime = 0.04f;

}  // namespace stdstates
