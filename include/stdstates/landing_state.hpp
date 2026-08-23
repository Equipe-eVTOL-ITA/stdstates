#pragma once

// LandingState — o nome antigo do pouso exponencial, mantido para as missoes
// que o usam (sae2026, ensaio_em_voo).
//
// Era uma segunda copia da mesma descida, com um bug proprio: media o tempo com
// duration_cast<seconds>, que trunca, e a 20 Hz isso fazia da "exponencial" uma
// escada de um degrau por segundo. Duas implementacoes da mesma coisa foi como
// o bug sobreviveu a propria correcao no PrecisionLandingState.
//
// O contrato nao mudou; `landing_timeout` inclusive voltou a ser lido.
//
// A exponencial e fixa aqui de proposito: quem escreveu `LandingState` no .cpp
// pediu a exponencial, e uma chave de YAML nao deve mudar isso pelas costas.
// Para escolher outra abordagem, use `landing_mode` -- ver landing/registro.hpp.

#include <string>

#include "stdstates/precision_landing_state.hpp"

class LandingState : public PrecisionLandingState
{
public:
  LandingState()
  : PrecisionLandingState(std::string("exponencial")) {}
};
