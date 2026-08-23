#pragma once

// Estrategia — a interface de uma abordagem de pouso.
//
// POR QUE ESTA CAMADA EXISTE
//
// Havia tres caminhos de pouso neste pacote, sem parentesco nenhum entre si:
// `LandingState` (exponencial, com o tempo truncado em segundos inteiros),
// `PrecisionLandingState` (a mesma exponencial, corrigida) e
// `LandAndDisarmState` (o modo LAND do proprio PX4). Escolher entre eles era
// escrever `std::make_unique<Concreto>()` no .cpp da missao -- ou seja, trocar
// de abordagem era recompilar, e comparar duas era recompilar duas vezes.
//
// Aqui a abordagem vira um valor: a chave `landing_mode` no YAML da missao.
// Acrescentar uma nova e um header e uma linha no registro.hpp.
//
// O CONTRATO
//
//   preparar()  roda uma vez, no on_enter. Le os parametros e calcula o que
//               depender da altura ATUAL. false = nao da para pousar assim, e
//               o estado devolve "ERROR" sem nunca comandar o drone.
//
//   passo()     roda a cada tick da FSM (50 ms, 20 Hz). Recebe o tempo em
//               segundos FRACIONARIOS desde o preparar() -- fracionarios
//               porque truncar para inteiro foi um bug real aqui: a velocidade
//               so mudava uma vez por segundo, e a descida "exponencial" era
//               na verdade uma escada de vinte comandos iguais e um degrau.
//
//   encerrar()  roda no on_exit, inclusive quando a FSM aborta por outro
//               caminho. Serve para parar o drone.
//
// O VOCABULARIO DE SAIDA E FIXO: "" (seguir), "LANDED", "ERROR".
//
// Isto nao e detalhe. A fsm::FSM LANCA uma excecao quando um estado devolve um
// outcome que nao esta nas suas transicoes. Se uma estrategia pudesse inventar
// um outcome proprio, trocar uma linha de YAML derrubaria a missao em pleno
// voo com "Outcome [X] doesn't belong to current state". Uma escolha de
// configuracao nao pode ter esse poder. Quem precisa distinguir "nao desarmou"
// de "falhou" usa o LandAndDisarmState direto, que tem outcome proprio para
// isso de proposito.

#include <memory>
#include <string>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

namespace stdstates::landing
{

/// Outcomes que uma estrategia pode devolver. Ver o comentario acima.
inline constexpr const char * kSeguir = "";
inline constexpr const char * kPousado = "LANDED";
inline constexpr const char * kErro = "ERROR";

class Estrategia
{
public:
  virtual ~Estrategia() = default;

  /// Le os parametros e prepara o pouso. false = nao da para pousar assim.
  virtual bool preparar(fsm::Blackboard & blackboard, const std::shared_ptr<Drone> & drone) = 0;

  /// Um tick da FSM. `t` = segundos fracionarios desde o preparar().
  virtual std::string passo(const std::shared_ptr<Drone> & drone, double t) = 0;

  /// Chamado ao sair do estado, por qualquer caminho.
  virtual void encerrar(const std::shared_ptr<Drone> & drone)
  {
    if (drone != nullptr) {drone->setLocalVelocity(0.0, 0.0, 0.0, 0.0);}
  }

  /// O nome pelo qual esta estrategia e escolhida no YAML.
  virtual const char * nome() const = 0;
};

// ---------------------------------------------------------------------------
// Helpers comuns as estrategias que descem por perfil de velocidade.
// ---------------------------------------------------------------------------

/// Folga sobre o tempo calculado, para cobrir a diferenca entre a velocidade
/// comandada e a que o controlador de fato atinge.
inline constexpr double kMargemSegurancaS = 5.0;

/// A altura do topo da base, como DISTANCIA POSITIVA acima da origem.
///
/// A convencao e FRD: para cima e negativo, entao `max_base_height` deveria
/// ser negativo. Metade do workspace escreve positivo -- cbr2026/fase1 e fase3
/// usam -1.5 e -0.2, enquanto fase4, sae2026, ensaio_em_voo e o gerador de
/// missoes usam 0.5.
///
/// Recusar o sinal errado quebraria missoes que voam hoje; aceita-lo em
/// silencio deixaria o perfil de descida ser calculado para uma queda que nao
/// e a real. Entao: aceita os dois, usa a magnitude -- que e igual nos dois
/// casos -- e AVISA, toda vez, com o valor e a correcao. O aviso sai no log da
/// missao, que e onde alguem depurando um pouso vai olhar.
inline float alturaDaBase(float max_base_height, const std::shared_ptr<Drone> & drone)
{
  if (max_base_height > 0.0f && drone != nullptr) {
    drone->log(
      "AVISO: max_base_height=" + std::to_string(max_base_height) +
      " esta POSITIVO. A convencao e FRD (para cima e negativo).");
    drone->log(
      "       Usando " + std::to_string(max_base_height) +
      " m como altura da base. Corrija o YAML para -" +
      std::to_string(max_base_height) + ".");
  }
  return std::abs(max_base_height);
}

}  // namespace stdstates::landing
