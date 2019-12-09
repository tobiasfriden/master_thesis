#ifndef ASTAR_H_
#define ASTAR_H_

#include <chrono>

#include "graph.h"
#include "hlut.h"
#include "constants.h"
#include "obstacle.h"

std::vector<Coordinate> astar(
    Simulator& sim,
    const MotionPrimitiveSet& primitives,
    const HLUT& hlut,
    const Obstacles& obst,
    const S2LatLng& origin,
    const Vector2_d& to,
    double goal_hdg,
    double inflation
);

std::vector<Coordinate> filter_solution(const Obstacles& obst, Simulator& sim, std::vector<Coordinate> pathVec);

#endif