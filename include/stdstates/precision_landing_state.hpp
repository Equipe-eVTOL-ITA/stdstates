#pragma once

// PrecisionLandingState — pousa usando a abordagem escolhida no YAML.
//
// Este estado era a descida exponencial. Agora ele e a CASCA: le
// `landing_mode` da blackboard, cria a estrategia correspondente e delega.
// A matematica exponencial esta em landing/exponencial.hpp, movida para la sem
// mudanca nenhuma de comportamento.
//
// POR QUE O NOME DA CLASSE NAO MUDOU
//
// `PrecisionLanding` e o nome com que o stdbt registra este estado na fabrica
// da Behavior Tree, e ele aparece nos XML das arvores -- que estao em outro
// repositorio. Um nome melhor nao vale quebrar toda arvore existente.
//
// Contrato com a blackboard:
//
//   opcional  "landing_mode"  std::string  "exponencial" (padrao) | "px4" | "s_curve"
//
//   Os demais parametros dependem do modo; cada header em landing/ documenta
//   os seus. O modo padrao continua lendo landing_velocity_max/min e
//   max_base_height, como sempre leu.
//
// Outcomes: ""        (descendo)
//           "LANDED"  (pouso concluido)
//           "ERROR"   (parametro ausente, incoerente, ou modo desconhecido)
//
// O vocabulario e FIXO, e nenhuma estrategia pode ampliá-lo: a fsm::FSM lanca
// excecao em outcome que nao esteja nas transicoes do estado, e uma troca de
// linha no YAML nao pode derrubar a missao em voo. Ver landing/estrategia.hpp.
//
// O que NAO vem para ca: em 2025 o `on_exit` deste estado registrava a base na
// lista, contava quantas faltavam e escrevia "finished_bases" na blackboard.
// Isso e contabilidade da fase 1 -- um estado generico de pouso nao pode saber
// o que e uma "base". A missao faz isso no seu proprio estado seguinte.

#include <chrono>
#include <memory>
#include <string>
#include <utility>   // std::move, no construtor de modo fixo

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

#include "stdstates/blackboard_params.hpp"
#include "stdstates/landing/registro.hpp"

class PrecisionLandingState : public fsm::State
{
public:
  PrecisionLandingState()
  : fsm::State() {}

  /// Fixa a abordagem no codigo, ignorando o `landing_mode` do YAML.
  ///
  /// Existe para o caso em que a escolha e da MISSAO, e nao de quem configura:
  /// uma fase cuja regra exige um pouso especifico nao deveria poder ser
  /// desconfigurada por engano. Sem argumento, o YAML manda.
  explicit PrecisionLandingState(std::string modo_fixo)
  : fsm::State(), modo_fixo_(std::move(modo_fixo)) {}

  void on_enter(fsm::Blackboard & blackboard) override
  {
    estrategia_.reset();

    auto drone_ptr = blackboard.get<std::shared_ptr<Drone>>("drone");
    if (drone_ptr == nullptr) {return;}
    drone_ = *drone_ptr;
    if (drone_ == nullptr) {return;}

    drone_->log("");
    drone_->log("STATE: PRECISION LANDING");

    std::string modo = modo_fixo_;
    if (modo.empty()) {
      modo = stdstates::optional<std::string>(
        blackboard, "landing_mode", stdstates::landing::kModoPadrao);
    }

    auto estrategia = stdstates::landing::criar(modo);
    if (estrategia == nullptr) {
      drone_->log("ERRO: landing_mode desconhecido: '" + modo + "'.");
      std::string lista;
      for (const auto & m : stdstates::landing::modos()) {
        lista += (lista.empty() ? "" : ", ") + m;
      }
      drone_->log("      Modos aceitos: " + lista);
      return;
    }

    if (!estrategia->preparar(blackboard, drone_)) {return;}

    estrategia_ = std::move(estrategia);
    start_time_ = std::chrono::steady_clock::now();
  }

  std::string act(fsm::Blackboard & blackboard) override
  {
    (void)blackboard;
    if (estrategia_ == nullptr) {return "ERROR";}

    // Em SEGUNDOS FRACIONARIOS. O codigo de 2025 usava
    // `duration_cast<std::chrono::seconds>`, que TRUNCA para inteiro: a
    // velocidade so mudava uma vez por segundo, o que a 20 Hz significa vinte
    // comandos identicos seguidos de um degrau. A descida "exponencial" era na
    // verdade uma escada, e o solavanco a cada degrau e exatamente o tipo de
    // coisa que se confunde com problema de sintonia do controlador.
    const std::chrono::duration<double> elapsed =
      std::chrono::steady_clock::now() - start_time_;

    return estrategia_->passo(drone_, elapsed.count());
  }

  void on_exit(fsm::Blackboard & blackboard) override
  {
    (void)blackboard;
    // Roda por qualquer caminho de saida, inclusive quando a missao aborta o
    // pouso. Cada estrategia sabe como parar: as que descem por setpoint zeram
    // a velocidade, e a do PX4 nao publica nada -- um setpoint depois do LAND
    // disputaria o controle com o firmware a centimetros do chao.
    if (estrategia_ != nullptr && drone_ != nullptr) {
      estrategia_->encerrar(drone_);
    }
  }

private:
  std::shared_ptr<Drone> drone_;
  std::unique_ptr<stdstates::landing::Estrategia> estrategia_;
  std::string modo_fixo_;
  std::chrono::steady_clock::time_point start_time_;
};
