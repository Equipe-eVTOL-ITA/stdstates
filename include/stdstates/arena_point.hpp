#pragma once

// Ponto de uma rota na arena.
//
// Esta struct estava duplicada em `itajuba/fase1`, `itajuba/fase3` e
// `sae_2025/fase3`, com definições idênticas e guardas de include diferentes
// (`ARENA_POINTS_CPP`). Fica aqui porque o `WaypointListState` a consome, e
// porque uma definição só evita que duas fases discordem sobre o que é um
// waypoint.

#include <Eigen/Eigen>

struct ArenaPoint
{
  Eigen::Vector3d coordinates;
  bool is_visited = false;
};
