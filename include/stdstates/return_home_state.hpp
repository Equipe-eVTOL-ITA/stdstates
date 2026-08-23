#pragma once

// ReturnHomeState — volta à posição de casa em duas fases: horizontal na
// altitude de cruzeiro, depois descida.
//
// Portado de `return_home_state.hpp` da fase 1 da CBR 2025, com três correções
// explicadas ao longo do arquivo.
//
// Contrato com a blackboard:
//
//   entrada   "home_position"           Eigen::Vector3d
//             "takeoff_height"          float  (FRD, negativo)
//             "max_horizontal_velocity" float
//             "position_tolerance"      float
//   opcional  "return_home_timeout"     float  (padrão: derivado da distância)
//
// Outcomes: ""         (voltando)
//           "AT HOME"  (chegou, ou o tempo acabou)
//           "ERROR"    (parâmetro ausente)
//
// O que NÃO vem para cá: em 2025 o `on_exit` chamava `drone->land()`, depois
// `rclcpp::sleep_for(5s)` e `disarmSync()`. O `on_exit` roda DENTRO do callback
// do timer da FSM — dormir cinco segundos ali trava o executor inteiro, o que
// também congela a telemetria e a visão. Quem quiser pousar e desarmar ao
// chegar deve fazê-lo num estado próprio, não num on_exit bloqueante.

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <Eigen/Eigen>

#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"

#include "stdstates/blackboard_params.hpp"
#include "stdstates/motion.hpp"

class ReturnHomeState : public fsm::State
{
public:
  ReturnHomeState()
  : fsm::State() {}

  void on_enter(fsm::Blackboard & blackboard) override
  {
    ok_ = false;

    // CORREÇÃO. Em 2025 este campo era inicializado apenas na declaração do
    // membro (`bool over_base = false;`), nunca no on_enter. Como a instância
    // do estado é criada uma vez e reutilizada a cada entrada, uma segunda
    // passagem pelo RETURN HOME começaria já com over_base = true e pularia
    // direto para a descida — descendo de onde estivesse, não em casa. Na fase
    // 1 o estado era terminal, então o bug nunca se manifestou; num estado
    // genérico ele se manifestaria.
    over_home_ = false;

    auto drone_ptr = blackboard.get<std::shared_ptr<Drone>>("drone");
    if (drone_ptr == nullptr) return;
    drone_ = *drone_ptr;
    if (drone_ == nullptr) return;

    drone_->log("");
    drone_->log("STATE: RETURN HOME");

    auto * home_ptr = blackboard.get<Eigen::Vector3d>("home_position");
    if (home_ptr == nullptr) {
      drone_->log("ERRO: parametro ausente na blackboard: 'home_position'");
      return;
    }

    float takeoff_height = 0.0f;
    if (!stdstates::require(blackboard, drone_, "takeoff_height", takeoff_height)) return;
    if (!stdstates::require(blackboard, drone_, "max_horizontal_velocity", max_velocity_)) return;
    if (!stdstates::require(blackboard, drone_, "position_tolerance", tolerance_)) return;

    goal_ = Eigen::Vector3d(home_ptr->x(), home_ptr->y(), takeoff_height);
    yaw_ = static_cast<float>(drone_->getOrientation()[2]);

    // CORREÇÃO. Em 2025 havia `if (elapsed_time > 15) return "AT HOME";` — um
    // literal, sem explicação. Numa arena maior que a da CBR 2025, ou com
    // velocidade menor, quinze segundos acabam ANTES de o drone chegar, e o
    // estado declara "cheguei em casa" no meio do caminho. O que vem depois é
    // um pouso onde quer que ele esteja.
    //
    // Aqui o limite é derivado da distância real que falta percorrer, com folga
    // generosa, e continua sobrescrevível pelo YAML.
    const Eigen::Vector3d pos = drone_->getLocalPosition();
    const double distancia = (goal_ - pos).norm();
    const double t_estimado =
      max_velocity_ > 0.0f ? distancia / max_velocity_ : 0.0;
    timeout_s_ = stdstates::optional<float>(
      blackboard, "return_home_timeout",
      static_cast<float>(t_estimado * kFatorDeFolga + kMargemFixaS));

    drone_->log(
      "Casa a " + std::to_string(distancia) + " m; timeout " +
      std::to_string(timeout_s_) + " s");

    // COMO voltar e escolha da missao: ver stdstates/motion.hpp.
    politica_ = stdstates::criarPolitica(blackboard, drone_);
    if (politica_ == nullptr) {return;}
    limites_ = stdstates::limitesDaBlackboard(blackboard, tolerance_);
    politica_->iniciar(goal_, yaw_);

    start_time_ = std::chrono::steady_clock::now();
    ok_ = true;
  }

  std::string act(fsm::Blackboard & blackboard) override
  {
    (void)blackboard;
    if (!ok_) return "ERROR";

    const std::chrono::duration<double> elapsed =
      std::chrono::steady_clock::now() - start_time_;

    if (elapsed.count() > timeout_s_) {
      drone_->setLocalVelocity(0.0, 0.0, 0.0, 0.0);
      drone_->log("Timeout no retorno; encerrando onde esta.");
      return "AT HOME";
    }

    const Eigen::Vector3d pos = drone_->getLocalPosition();

    // Fase 1: alcançar a vertical de casa mantendo a altitude atual. Separar as
    // duas fases evita descer sobre um obstáculo no caminho.
    if (!over_home_) {
      const Eigen::Vector3d horizontal_goal(goal_.x(), goal_.y(), pos.z());
      const Eigen::Vector3d diff = horizontal_goal - pos;

      if (diff.norm() < tolerance_) {
        over_home_ = true;
        // Destino novo: a politica axial precisa reavaliar a proa.
        politica_->iniciar(goal_, yaw_);
        drone_->log("Sobre a casa; iniciando descida.");
        return "";
      }
      politica_->irPara(drone_, horizontal_goal, yaw_, limites_);
      return "";
    }

    // Fase 2: descer até a altitude de casa.
    const Eigen::Vector3d diff = goal_ - pos;
    if (diff.norm() < tolerance_) {
      drone_->setLocalVelocity(0.0, 0.0, 0.0, 0.0);
      drone_->log("Em casa.");
      return "AT HOME";
    }
    politica_->irPara(drone_, goal_, yaw_, limites_);
    return "";
  }

private:
  /// Folga sobre o tempo ideal de percurso: o drone acelera e desacelera, e o
  /// passo limitado não atinge a velocidade máxima o tempo todo.
  static constexpr double kFatorDeFolga = 3.0;
  static constexpr double kMargemFixaS = 10.0;

  std::shared_ptr<Drone> drone_;
  std::unique_ptr<drone::MotionPolicy> politica_;
  drone::Limites limites_;
  Eigen::Vector3d goal_ = Eigen::Vector3d::Zero();
  float max_velocity_ = 0.0f;
  float tolerance_ = 0.0f;
  float yaw_ = 0.0f;
  bool over_home_ = false;
  double timeout_s_ = 0.0;
  std::chrono::steady_clock::time_point start_time_;
  bool ok_ = false;
};
