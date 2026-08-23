#pragma once

// registro.hpp — o mapa de `landing_mode` para a estrategia.
//
// Abordagem nova: um header ao lado destes, um include, e uma linha no criar().
// A fase4 (Behavior Tree) ganha junto, porque o stdbt embrulha o mesmo estado.
//
// OS NOMES SAO INTERFACE PUBLICA: aparecem nos config/*.yaml de outros
// repositorios, e renomear um quebra o YAML sem erro de compilacao.

#include <memory>
#include <string>
#include <vector>

#include "stdstates/landing/estrategia.hpp"
#include "stdstates/landing/exponencial.hpp"
#include "stdstates/landing/px4_land.hpp"
#include "stdstates/landing/s_curve.hpp"

// >>> CONTRATO pouso.modos
// A abordagem de pouso e uma chave de YAML, e nao codigo:
//
//     landing_mode: exponencial   # padrao -- v(t) = v_max·e^(-t/tau)
//     landing_mode: px4           # o modo LAND do firmware; TIRA de offboard
//                                 #   e DESARMA; ignora landing_velocity_*
//     landing_mode: s_curve       # perfil em S, sem solavanco nas pontas
//
// Parametros do exponencial e do s_curve: landing_velocity_max/min,
// max_base_height, landing_timeout (folga, padrao 5 s).
// Parametros do px4: disarm_grace (3 s), disarm_timeout (20 s).
//
// max_base_height deveria ser NEGATIVO (FRD, para cima e negativo). Metade do
// workspace escreve positivo; os dois sao aceitos, a magnitude e usada, e sai
// um aviso no log da missao.
//
// A altura de PARTIDA nunca e parametro: e medida ao entrar no estado.
// <<< CONTRATO

namespace stdstates::landing
{

/// Padrao: o que todas as missoes faziam antes desta camada. Mudar aqui muda
/// o pouso de todo mundo de uma vez.
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
inline std::vector<std::string> modos()
{
  return {"exponencial", "px4", "s_curve"};
}

}  // namespace stdstates::landing
