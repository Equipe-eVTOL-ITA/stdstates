// Teste de compilação dos estados.
//
// POR QUE ISTO EXISTE
//
// `stdstates` é header-only. Isso significa que `colcon build --packages-select
// stdstates` termina em um segundo SEM COMPILAR NENHUM HEADER: ele só copia os
// arquivos para o install. Um erro de sintaxe, um include faltando ou uma
// assinatura de método fora do contrato do `fsm::State` passariam pelo CI
// intactos, e só apareceriam quando uma missão tentasse usar o estado — em
// geral na véspera da competição.
//
// Este arquivo é o consumidor que força a instanciação. Ele não verifica
// comportamento em voo (para isso é preciso SITL); verifica que os estados
// EXISTEM, compilam, e satisfazem a interface que a FSM espera.

#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "fsm/fsm.hpp"

#include "stdstates/arena_point.hpp"
#include "stdstates/blackboard_params.hpp"
#include "stdstates/next_waypoints.hpp"
#include "stdstates/precision_align_state.hpp"
#include "stdstates/precision_landing_state.hpp"
#include "stdstates/return_home_state.hpp"

// --- a interface exigida pela FSM -------------------------------------------

// Se algum estado deixar de derivar de fsm::State, ou mudar a assinatura de
// act/on_enter, isto falha na compilação e não em voo.
static_assert(std::is_base_of<fsm::State, WaypointListState>::value, "");
static_assert(std::is_base_of<fsm::State, PrecisionAlignState>::value, "");
static_assert(std::is_base_of<fsm::State, PrecisionLandingState>::value, "");
static_assert(std::is_base_of<fsm::State, ReturnHomeState>::value, "");

TEST(Compilacao, EstadosSaoConstruiveisEArmazenaveis)
{
  // A FSM guarda os estados como std::unique_ptr<fsm::State>; construir assim
  // é o que `add_state` faz.
  std::vector<std::unique_ptr<fsm::State>> estados;
  estados.push_back(std::make_unique<WaypointListState>());
  estados.push_back(std::make_unique<PrecisionAlignState>());
  estados.push_back(std::make_unique<PrecisionLandingState>());
  estados.push_back(std::make_unique<ReturnHomeState>());

  EXPECT_EQ(estados.size(), 4u);
  for (const auto & e : estados) {
    EXPECT_NE(e, nullptr);
  }
}

// --- o comportamento que dá para testar sem drone ---------------------------

TEST(BlackboardParams, RequireFalhaEmVezDeExplodirComChaveAusente)
{
  fsm::Blackboard bb;
  float destino = -1.0f;

  // O ponto todo do helper: uma chave ausente devolve false. O idioma que ele
  // substitui — `*bb.get<float>("nao_existe")` — seria dereference de nullptr.
  const bool achou = stdstates::require(bb, nullptr, "nao_existe", destino);

  EXPECT_FALSE(achou);
  EXPECT_FLOAT_EQ(destino, -1.0f) << "o destino nao deve ser tocado quando falta a chave";
}

TEST(BlackboardParams, RequireLeValorPresente)
{
  fsm::Blackboard bb;
  bb.set<float>("tolerancia", 0.25f);

  float destino = 0.0f;
  EXPECT_TRUE(stdstates::require(bb, nullptr, "tolerancia", destino));
  EXPECT_FLOAT_EQ(destino, 0.25f);
}

TEST(BlackboardParams, OptionalUsaOPadraoQuandoFalta)
{
  fsm::Blackboard bb;
  EXPECT_FLOAT_EQ(stdstates::optional<float>(bb, "ausente", 42.0f), 42.0f);

  bb.set<float>("presente", 7.0f);
  EXPECT_FLOAT_EQ(stdstates::optional<float>(bb, "presente", 42.0f), 7.0f);
}

TEST(BlackboardParams, SampleTimeDosPidsFolgaSobreOTimerDaFsm)
{
  // A FSM roda a 20 Hz (50 ms). PidController::compute() devolve 0.0f quando
  // chamado antes de sample_time. Se os dois forem iguais, o jitter produz
  // zeros intermitentes no controle. Este teste trava a folga.
  constexpr float periodo_da_fsm = 0.05f;
  EXPECT_LT(stdstates::kPidSampleTime, periodo_da_fsm)
    << "sample_time do PID precisa ser menor que o periodo do timer da FSM";
}

TEST(ArenaPoint, NasceNaoVisitado)
{
  ArenaPoint p{Eigen::Vector3d(1.0, 2.0, -3.0), false};
  EXPECT_FALSE(p.is_visited);

  ArenaPoint padrao{Eigen::Vector3d::Zero(), false};
  EXPECT_FALSE(padrao.is_visited);
}
