#ifndef WAYPOINT_OPT_H_
#define WAYPOINT_OPT_H_

#include <s2/s2latlng.h>
#include <s2/s2earth.h>
#include <Eigen/Dense>
#include <vector>

#include "simulation.h"
#include "obstacle.h"
#include "graph.h"

class WaypointOpt {
public:
    WaypointOpt(double init_hdg, double goal_hdg) : _init_hdg{init_hdg}, _goal_hdg{goal_hdg} {};

    std::vector<Coordinate> improve_path(
        Simulator& sim,
        const Obstacles& obst,
        const std::vector<Coordinate>& path
    );

private:
    double get_mission_J(Simulator& sim, const Obstacles& obst, const std::vector<Vector2_d>& mission);
    void compute_gradient(Simulator& sim, const Obstacles& obst, Eigen::VectorXd & GradJ, std::vector<Vector2_d> controlPoints, double h);
    std::vector<Vector2_d> filter_mission(std::vector<Vector2_d> mission);

    double _init_hdg;
    double _goal_hdg;
    double _terminal_scale;

};

#endif