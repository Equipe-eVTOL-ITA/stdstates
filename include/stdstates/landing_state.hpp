#pragma once

// LandingState — o nome antigo do pouso, mantido para as missoes que o usam.
//
// POR QUE ELE VIROU UM ALIAS
//
// Havia DUAS descidas exponenciais neste pacote, lendo as mesmas chaves da
// blackboard e fazendo a mesma conta: esta e a do PrecisionLandingState. A
// diferenca era um bug.
//
// Esta media o tempo com `duration_cast<std::chrono::seconds>`, que TRUNCA
// para inteiro. A 20 Hz isso significa vinte comandos de velocidade iguais e
// um degrau, uma vez por segundo: a descida "exponencial" era uma escada, e o
// solavanco a cada degrau se confunde com problema de sintonia do controlador.
// O PrecisionLandingState ja tinha sido corrigido; esta copia continuava a
// voar em sae2026 (as cinco missoes) e em ensaio_em_voo.
//
// Manter duas implementacoes da mesma coisa e como o bug sobreviveu a propria
// correcao. Agora ha uma, e este nome aponta para ela.
//
// O CONTRATO NAO MUDOU:
//
//   entrada  "landing_velocity_max"  float
//            "landing_velocity_min"  float
//            "max_base_height"       float
//   opcional "landing_timeout"       float  (folga em s; padrao 5)
//
//   Outcomes: "" / "LANDED" / "ERROR" -- os mesmos de antes.
//
// O `landing_timeout` inclusive volta a ser LIDO: o PrecisionLandingState o
// ignorava, usando 5 s fixos, enquanto quatro pacotes o declaravam no YAML
// acreditando que servia para alguma coisa.
//
// Para escolher outra abordagem de pouso (o modo LAND do PX4, ou o perfil em
// S), use `landing_mode` no YAML -- ver stdstates/landing/registro.hpp. Esta
// classe fixa a exponencial de proposito: quem escreveu `LandingState` no .cpp
// da missao pediu a exponencial, e uma chave de YAML nao deve mudar isso pelas
// costas.

#include <string>

#include "stdstates/precision_landing_state.hpp"

class LandingState : public PrecisionLandingState
{
public:
  LandingState()
  : PrecisionLandingState(std::string("exponencial")) {}
};
