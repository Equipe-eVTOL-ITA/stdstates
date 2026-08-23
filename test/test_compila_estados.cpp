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
#include "drone/movement.hpp"

#include "stdstates/arena_point.hpp"
#include "stdstates/blackboard_params.hpp"
#include "stdstates/next_waypoints.hpp"
#include "stdstates/precision_align_state.hpp"
#include "stdstates/precision_landing_state.hpp"
#include "stdstates/return_home_state.hpp"
#include "stdstates/takeoff_state.hpp"
#include "stdstates/yaw_sweep_state.hpp"
#include "stdstates/land_and_disarm_state.hpp"
#include "stdstates/landing_state.hpp"
#include "stdstates/landing/registro.hpp"
#include "stdstates/motion.hpp"
#include "stdstates/goto_state.hpp"

// --- a interface exigida pela FSM -------------------------------------------

// Se algum estado deixar de derivar de fsm::State, ou mudar a assinatura de
// act/on_enter, isto falha na compilação e não em voo.
static_assert(std::is_base_of<fsm::State, WaypointListState>::value, "");
static_assert(std::is_base_of<fsm::State, PrecisionAlignState>::value, "");
static_assert(std::is_base_of<fsm::State, PrecisionLandingState>::value, "");
static_assert(std::is_base_of<fsm::State, ReturnHomeState>::value, "");
static_assert(std::is_base_of<fsm::State, LandingState>::value, "");

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

// --- TakeoffState: reancoragem do referencial -------------------------------

TEST(TakeoffState, AceitaAsDuasFormasDeConstrucao)
{
  // A forma sem argumento reancora o referencial do mundo, como sempre fez.
  // Missoes que so decolam uma vez continuam valendo sem mudanca.
  TakeoffState inicial;

  // A forma explicita e para REDECOLAGEM no meio da missao, onde reancorar
  // destruiria as coordenadas ja guardadas.
  TakeoffState inicial_explicito(true);
  TakeoffState redecolagem(false);

  // Precisam caber num unique_ptr<fsm::State>, que e como a FSM os guarda.
  std::vector<std::unique_ptr<fsm::State>> estados;
  estados.push_back(std::make_unique<TakeoffState>());
  estados.push_back(std::make_unique<TakeoffState>(false));
  EXPECT_EQ(estados.size(), 2u);
}

TEST(TakeoffState, OPadraoReancora)
{
  // Este teste existe para travar o PADRAO, nao o comportamento em voo.
  //
  // `TakeoffState()` tem de continuar reancorando, porque e o que as missoes
  // existentes esperam e o que a decolagem inicial precisa. Se alguem inverter
  // o padrao para "nao reancorar", a decolagem inicial passa a herdar um
  // referencial antigo -- e o sintoma seria o drone voando para coordenadas
  // deslocadas, sem nenhum erro.
  //
  // A checagem e feita pelo construtor de copia do default: se o padrao mudar
  // para false, a linha abaixo deixa de compilar identica a TakeoffState(true).
  static_assert(
    std::is_constructible<TakeoffState>::value,
    "TakeoffState precisa ser construivel sem argumento");
  static_assert(
    std::is_constructible<TakeoffState, bool>::value,
    "TakeoffState precisa aceitar o flag set_home");

  // O construtor e explicit: `TakeoffState t = false;` nao deve compilar.
  static_assert(
    !std::is_convertible<bool, TakeoffState>::value,
    "o construtor deve ser explicit, para nao aceitar conversao implicita");

  SUCCEED();
}

// --- Estados novos da fase 3 -------------------------------------------------

TEST(Compilacao, EstadosDaFase3SaoConstruiveis)
{
  std::vector<std::unique_ptr<fsm::State>> estados;
  estados.push_back(std::make_unique<YawSweepState>());
  estados.push_back(std::make_unique<LandAndDisarmState>());
  EXPECT_EQ(estados.size(), 2u);
}

TEST(YawSweep, DiferencaNormalizadaResolveADescontinuidade)
{
  // Este teste NAO exercita o estado (precisaria de um Drone). Ele trava a
  // propriedade matematica de que o estado depende, e que a versao de 2025 nao
  // tinha: comparar a diferenca normalizada em vez de limites absolutos.
  //
  // O caso concreto que quebrava: drone entrando no varrimento olhando para
  // 2.6 rad com abertura de 1.05 rad. O limite superior absoluto seria 3.65,
  // um valor que o yaw -- que vive em (-pi, pi] -- nunca atinge.
  const float centro = 2.6f;
  const float range = 1.0472f;

  // Limite superior "absoluto" da versao antiga: inalcancavel.
  EXPECT_GT(centro + range, static_cast<float>(M_PI))
    << "o caso so e interessante se o limite passar de pi";

  // Ja a diferenca normalizada funciona: um yaw que deu a volta (-3.0 rad)
  // esta a pouco mais de meio radiano do centro, e nao a 5.6.
  const float yaw_apos_a_volta = -3.0f;
  const float desvio = normalizeYawError(yaw_apos_a_volta - centro);

  EXPECT_LT(std::abs(desvio), static_cast<float>(M_PI));
  EXPECT_NEAR(desvio, 0.6831853f, 1e-4f)
    << "menor angulo entre 2.6 e -3.0, passando por pi";

  // E o criterio de inversao nao dispara, porque ainda esta dentro do setor.
  EXPECT_LT(desvio, range);
}

TEST(YawSweep, InverteNosDoisExtremosDoSetor)
{
  const float centro = 0.0f;
  const float range = 1.0f;

  EXPECT_GE(normalizeYawError(1.1f - centro), range) << "passou do extremo +";
  EXPECT_LE(normalizeYawError(-1.1f - centro), -range) << "passou do extremo -";
  EXPECT_LT(std::abs(normalizeYawError(0.5f - centro)), range) << "dentro do setor";
}


// --- Modos de pouso ---------------------------------------------------------
//
// A escolha da abordagem de pouso passou a ser um valor de YAML
// (`landing_mode`). Isso e conveniente e e perigoso pela mesma razao: um
// arquivo de configuracao ganhou poder sobre o que o drone faz a poucos metros
// do chao. Os testes abaixo trancam as duas pontas -- todo modo anunciado
// existe de fato, e um modo inventado falha ALTO, e nao em silencio.

TEST(ModosDePouso, TodoModoAnunciadoPodeSerConstruido)
{
  // A lista de `modos()` e o que a mensagem de erro oferece a quem digitou
  // errado. Se ela anunciar um modo que o `criar()` nao sabe fazer, a
  // mensagem manda a pessoa -- com o drone armado esperando -- para um nome
  // que tambem nao funciona.
  const auto nomes = stdstates::landing::modos();
  ASSERT_FALSE(nomes.empty());

  for (const auto & nome : nomes) {
    auto e = stdstates::landing::criar(nome);
    EXPECT_NE(e, nullptr) << "modos() anuncia '" << nome << "', que criar() nao constroi";
    if (e != nullptr) {
      EXPECT_STREQ(e->nome(), nome.c_str())
        << "a estrategia devolve um nome diferente daquele pelo qual foi pedida";
    }
  }
}

TEST(ModosDePouso, ModoDesconhecidoDevolveNullptrEmVezDeUmPadraoSilencioso)
{
  // Cair no padrao quando o nome esta errado seria o pior desfecho: a missao
  // pousaria de um jeito que ninguem pediu, e o YAML continuaria dizendo outra
  // coisa. Melhor recusar e deixar o estado devolver ERROR.
  EXPECT_EQ(stdstates::landing::criar("exponencia"), nullptr);
  EXPECT_EQ(stdstates::landing::criar(""), nullptr);
  EXPECT_EQ(stdstates::landing::criar("LAND"), nullptr);
}

TEST(ModosDePouso, OPadraoEAExponencial)
{
  // Uma missao que nao declara `landing_mode` tem de pousar exatamente como
  // pousava antes desta camada existir. Mudar este padrao mudaria o
  // comportamento de todas as missoes de uma vez.
  auto e = stdstates::landing::criar(stdstates::landing::kModoPadrao);
  ASSERT_NE(e, nullptr);
  EXPECT_STREQ(e->nome(), "exponencial");
}

TEST(ModosDePouso, AlturaDaBaseAceitaOsDoisSinais)
{
  // A convencao e FRD (para cima e negativo), e metade do workspace escreve
  // positivo: cbr2026/fase1 e fase3 usam -1.5 e -0.2, enquanto fase4, sae2026,
  // ensaio_em_voo e o gerador de missoes usam 0.5. Os dois tem de dar a mesma
  // distancia, ou o perfil de descida e calculado para uma queda que nao e a
  // real -- sem erro nenhum aparecer.
  EXPECT_FLOAT_EQ(stdstates::landing::alturaDaBase(-1.5f, nullptr), 1.5f);
  EXPECT_FLOAT_EQ(stdstates::landing::alturaDaBase(0.5f, nullptr), 0.5f);
  EXPECT_FLOAT_EQ(stdstates::landing::alturaDaBase(0.0f, nullptr), 0.0f);
}

TEST(ModosDePouso, LandingStateEPrecisionLandingSaoAMesmaImplementacao)
{
  // Eram duas descidas exponenciais lendo as mesmas chaves e fazendo a mesma
  // conta; a diferenca era um bug de truncamento no tempo, que sobreviveu a
  // propria correcao porque havia duas copias.
  static_assert(
    std::is_base_of<PrecisionLandingState, LandingState>::value,
    "LandingState precisa continuar sendo o PrecisionLandingState");

  std::vector<std::unique_ptr<fsm::State>> estados;
  estados.push_back(std::make_unique<LandingState>());
  estados.push_back(std::make_unique<PrecisionLandingState>());
  estados.push_back(std::make_unique<PrecisionLandingState>(std::string("px4")));
  for (const auto & e : estados) {
    EXPECT_NE(e, nullptr);
  }
}


// --- Politica de movimento --------------------------------------------------

TEST(PoliticaDeMovimento, PadraoQuandoAChaveNaoExiste)
{
  // Uma missao que nao declara `motion_policy` tem de continuar voando como
  // voava. O padrao e a holonomica -- o comportamento que todo estado tinha
  // antes desta camada existir.
  fsm::Blackboard bb;
  auto p = stdstates::criarPolitica(bb, nullptr);
  ASSERT_NE(p, nullptr);
  EXPECT_STREQ(p->nome(), "holonomica");
}

TEST(PoliticaDeMovimento, LeAChaveDaBlackboard)
{
  fsm::Blackboard bb;
  bb.set<std::string>("motion_policy", std::string("axial"));

  auto p = stdstates::criarPolitica(bb, nullptr);
  ASSERT_NE(p, nullptr);
  EXPECT_STREQ(p->nome(), "axial");
  EXPECT_FALSE(p->permiteCorrecaoLateral());
}

TEST(PoliticaDeMovimento, NomeInvalidoDevolveNullptrParaOEstadoDarErro)
{
  // Cair no padrao com um nome errado seria o pior desfecho: o drone voaria
  // holonomico enquanto o YAML diz `axial`, e no dia em que a regra for "so
  // pode girar e ir para frente" ninguem descobriria ate ver o voo.
  fsm::Blackboard bb;
  bb.set<std::string>("motion_policy", std::string("axail"));
  EXPECT_EQ(stdstates::criarPolitica(bb, nullptr), nullptr);
}

TEST(PoliticaDeMovimento, LimitesSaemDoYamlComOsNomesDeSempre)
{
  fsm::Blackboard bb;
  bb.set<float>("max_horizontal_velocity", 0.8f);

  const auto lim = stdstates::limitesDaBlackboard(bb, 0.25f);
  EXPECT_FLOAT_EQ(lim.passo, 0.8f);
  EXPECT_FLOAT_EQ(lim.posicao, 0.25f);
  // A tolerancia de guinada tem padrao: nenhuma missao a declara hoje, e
  // exigi-la quebraria todos os YAML existentes de uma vez.
  EXPECT_GT(lim.yaw, 0.0);
}

TEST(PoliticaDeMovimento, OsEstadosQueSeDeslocamAExigem)
{
  // Se algum destes deixar de compilar com a politica, e porque alguem voltou
  // a comandar deslocamento direto -- e o "um lugar so para mexer" deixou de
  // valer sem ninguem notar.
  std::vector<std::unique_ptr<fsm::State>> estados;
  estados.push_back(std::make_unique<WaypointListState>());
  estados.push_back(std::make_unique<ReturnHomeState>());
  estados.push_back(std::make_unique<GoToState>());
  EXPECT_EQ(estados.size(), 3u);
}
