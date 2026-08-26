#pragma once

// registro.hpp — o mapa de `takeoff_mode` para a estrategia.
//
// Abordagem nova: um header ao lado destes, um include, e uma linha no criar().
//
// OS NOMES SAO INTERFACE PUBLICA: aparecem nos config/*.yaml de outros
// repositorios, e renomear um quebra o YAML sem erro de compilacao.

#include <memory>
#include <string>
#include <vector>

#include "stdstates/takeoff/estrategia.hpp"
#include "stdstates/takeoff/px4_takeoff.hpp"
#include "stdstates/takeoff/stdtakeoff.hpp"

// >>> CONTRATO decolagem.modos
// A abordagem de decolagem e uma chave de YAML, e nao codigo:
//
//     takeoff_mode: stdtakeoff   # padrao -- sobe por setpoint, em OFFBOARD,
//                                #   ate o `takeoff_height` da missao
//     takeoff_mode: px4          # AUTO_TAKEOFF do firmware; sobe ate o
//                                #   MIS_TAKEOFF_ALT do PX4 e IGNORA o
//                                #   takeoff_height; sai de offboard e o
//                                #   estado reentra ao terminar
//
// Parametros do stdtakeoff: takeoff_height (FRD, negativo),
// max_vertical_velocity, position_tolerance.
// Parametros do px4: takeoff_timeout (30 s).
//
// A reancoragem do referencial (setHomePosition) NAO e da estrategia: e do
// TakeoffState, e vale para as duas. Ver o contrato px4.reancoragem-do-home.
// <<< CONTRATO

namespace stdstates::takeoff
{

/// O modo usado quando a missao nao declara `takeoff_mode`: o que todas as
/// missoes faziam antes desta camada.
inline constexpr const char * kModoPadrao = "stdtakeoff";

/// Cria a estrategia de um modo. nullptr = nome desconhecido.
inline std::unique_ptr<Estrategia> criar(const std::string & modo)
{
  if (modo == "stdtakeoff" || modo == "padrao" || modo == "offboard") {
    return std::make_unique<StdTakeoff>();
  }
  if (modo == "px4" || modo == "px4_takeoff") {
    return std::make_unique<Px4Takeoff>();
  }
  return nullptr;
}

/// Os nomes aceitos, para a mensagem de erro.
inline std::vector<std::string> modos()
{
  return {"stdtakeoff", "px4"};
}

}  // namespace stdstates::takeoff
