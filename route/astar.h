#ifndef ASTAR_H_
#define ASTAR_H_

#include <chrono>

#include <s2/s2latlng.h>

#include "graph.h"
#include "hlut.h"
#include "constants.h"

// class Grid {
// public:
//     Grid(
//         S2CrossingEdgeQuery* q,
//         S2ClosestEdgeQuery* dq,
//         int32 step
//     ) : _query{q}, _dQuery{dq}, step{step}, _inc{
//         {0, step},
//         {0, -step},
//         {step, 0},
//         {-step, 0},
//         {step, step},
//         {-step, step},
//         {-step, -step},
//         {step, -step}
//     } {};
//     int32 step;

//     bool isCrossing(Vector2_d const& ll1, Vector2_d const& ll2) const;
//     std::vector<Vector2_d> getNeighbours(const Vector2_d& c) const;

// private:
//     S2CrossingEdgeQuery* _query = nullptr;
//     S2ClosestEdgeQuery* _dQuery = nullptr;
//     int32 _inc[8][2];
// };


std::vector<Coordinate> astar(
    Simulator& sim,
    const MotionPrimitiveSet& primitives,
    const HLUT& hlut,
    const S2LatLng& origin,
    const Vector2_d& to,
    double goal_hdg,
    double inflation
);

std::vector<Coordinate> filter_solution(std::vector<Coordinate> pathVec);
#endif