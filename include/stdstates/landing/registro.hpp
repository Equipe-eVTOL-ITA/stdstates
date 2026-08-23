#pragma once

// registro.hpp — o mapa de `landing_mode` para a estrategia correspondente.
//
// PARA ACRESCENTAR UMA ABORDAGEM NOVA
//
//   1. escreva o header dela ao lado destes, herdando de Estrategia;
//   2. inclua-o aqui;
//   3. acrescente UMA linha na tabela abaixo.
//
// E so. A missao passa a poder escolhe-la pelo YAML, e a fase4 -- que roda
// Behavior Tree, nao FSM -- ganha junto, porque o stdbt embrulha o mesmo
// fsm::State.
//
// OS NOMES SAO INTERFACE PUBLICA
//
// Eles aparecem nos config/*.yaml das missoes, que estao em outros
// repositorios. Renomear um quebra todo YAML que o cita, sem erro de
// compilacao e sem aviso -- a missao sobe, nao acha o modo, e cai no
// tratamento de erro no meio do voo. Trate-os como o que sao. E a mesma regra
// que o stdbt/registrar.hpp aplica aos nomes dos nos da arvore.

#include <memory>
#include <string>
#include <vector>

#include "stdstates/landing/estrategia.hpp"
#include "stdstates/landing/exponencial.hpp"
#include "stdstates/landing/px4_land.hpp"
#include "stdstates/landing/s_curve.hpp"

namespace stdstates::landing
{

/// O modo usado quando a missao nao declara `landing_mode`.
///
/// E a exponencial porque e o que todas as missoes faziam antes desta camada
/// existir: uma missao que nao muda o YAML tem de pousar exatamente como
/// pousava. Mudar este padrao mudaria o comportamento de todo mundo de uma vez.
inline constexpr const char * kModoPadrao = "exponencial";

/// Cria a estrategia de um modo. nullptr = nome desconhecido.
inline std::unique_ptr<Estrategia> criar(const std::string & modo)
{
  if (modo == "exponencial" || modo == "exponential") {
    return std::make_unique<Exponencial>();
  }
  if (modo == "px4" || modo == "px4_land") {
    return std::make_unique<Px4Land>();
  }
  if (modo == "s_curve" || modo == "scurve") {
    return std::make_unique<SCurve>();
  }
  return nullptr;
}

/// Os nomes aceitos, para a mensagem de erro.
///
/// Recusar um modo desconhecido sem dizer quais existem obriga a abrir o
/// codigo -- e quem digitou "exponencia" no YAML esta, quase sempre, com o
/// drone armado esperando.
inline std::vector<std::string> modos()
{
  return {"exponencial", "px4", "s_curve"};
}

}  // namespace stdstates::landing
