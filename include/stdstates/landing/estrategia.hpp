#pragma once

// Estrategia — a interface de uma abordagem de pouso.
//
// A abordagem e um valor (`landing_mode` no YAML), e nao um
// std::make_unique<Concreto> no .cpp da missao. Ver landing/registro.hpp.
//
// O vocabulario de saida e FIXO: "" / "LANDED" / "ERROR". A fsm::FSM lanca
// excecao em outcome fora das transicoes do estado, e uma troca de linha no
// YAML nao pode derrubar a missao em voo.

#include <memory>
#include <string>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

namespace stdstates::landing
{

inline constexpr const char * kSeguir = "";
inline constexpr const char * kPousado = "LANDED";
inline constexpr const char * kErro = "ERROR";

class Estrategia
{
public:
  virtual ~Estrategia() = default;

  /// Roda no on_enter. false = nao da para pousar assim, e o estado devolve
  /// "ERROR" sem nunca comandar o drone.
  virtual bool preparar(fsm::Blackboard & blackboard, const std::shared_ptr<Drone> & drone) = 0;

  /// Um tick da FSM. `t` = segundos FRACIONARIOS desde o preparar(); truncar
  /// para inteiro ja transformou a exponencial numa escada.
  virtual std::string passo(const std::shared_ptr<Drone> & drone, double t) = 0;

  /// Chamado ao sair do estado, por qualquer caminho.
  virtual void encerrar(const std::shared_ptr<Drone> & drone)
  {
    if (drone != nullptr) {drone->setLocalVelocity(0.0, 0.0, 0.0, 0.0);}
  }

  /// O nome pelo qual esta estrategia e escolhida no YAML.
  virtual const char * nome() const = 0;
};

// ── Helpers das estrategias que descem por perfil de velocidade ─────────────

/// Folga sobre o tempo calculado, para cobrir a diferenca entre a velocidade
/// comandada e a que o controlador de fato atinge.
inline constexpr double kMargemSegurancaS = 5.0;

/// A altura do topo da base como distancia POSITIVA acima da origem.
///
/// A convencao e FRD (para cima e negativo), mas metade do workspace escreve
/// positivo. Recusar quebraria missoes que voam hoje; aceitar em silencio
/// calcularia o perfil para uma queda que nao e a real. Entao: usa a
/// magnitude, e avisa no log.
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
