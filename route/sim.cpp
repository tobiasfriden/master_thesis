#include <iostream>

#include <s2/s2latlng.h>

#include "Evaluator.hpp"

#include "simulation.h"
#include "opt.h"


Vector2_i grid_search(int num_cells, int step, float wind_dir, float goal_hdg){
    int best_x = step;
    int best_y = step;
    float cost;
    float best_cost = std::numeric_limits<float>().max();
    float best_yaw = 0;

    Vector2_d start(0, 0);
    Vector2_d goal;

    std::ofstream out("../grid_search.txt");

    NOMAD::Display disp(std::cout);
    NOMAD::Parameters p(disp);
    set_params(p);
    Vector2_d offset(cosf(to_rad(goal_hdg)), sinf(to_rad(goal_hdg)));
    offset *= 100;
    NOMAD::Point x0(2);
    x0[0] = offset[0];
    x0[1] = fmax(offset[1], 0);
    p.set_X0(x0);
    p.check();

    SimEvaluator se(p);
    NOMAD::Mads mads(p, &se);
    se.start_logging();
    se.set_goal(goal_hdg, Constants::hdg_error(), Constants::xtrack_error());
    mads.run();

    Simulator sim;
    for(int i=-num_cells; i<=num_cells; i++){
        for(int j=1; j<=num_cells; j++){
            sim.reset(0, 0, 0, 0);
            goal = Vector2_d(i*step, j*step);
            cost = sim.simulate_waypoints(start, goal);
            cost += fabs(sim.path_bearing() - goal_hdg)*Constants::hdg_w();
            cost += sim.xtrack_error()*Constants::xtrack_w();
            if(fabs(sim.path_bearing() - goal_hdg) < Constants::hdg_error() && fabs(sim.xtrack_error()) < Constants::xtrack_error()){
                out << cost << " ";
            } else {
                out << -1 << " ";
            }
            if(std::fabs(sim.path_bearing() - goal_hdg) < 10 &&
               std::fabs(sim.xtrack_error()) < 10 &&
               cost < best_cost){
                best_cost = cost;
                best_x = step*i;
                best_y = step*j;
                best_yaw = sim.yaw();
            }
        }
        out << std::endl;
    }
    std::cout << "best x: " << best_x << std::endl;
    std::cout << "best y: " << best_y << std::endl;
    std::cout << "best yaw: " << best_yaw << std::endl;
    std::cout << "best cost: " << best_cost << std::endl;
    se.stop_logging();
    return Vector2_i(best_x, best_y);
}

int main(int argc, char** argv, char** envp){
    // int num_of_cells = atoi(argv[1]);
    // int step = atoi(argv[2]);
    // int wind_dir = atof(argv[3]);
    // int goal_hdg = atof(argv[4]);
    // Vector2_i best = grid_search(num_of_cells, step, wind_dir, goal_hdg);
    // std::cout << best << std::endl;
    // Vector2_i best(100, 0);
    // std::cout << "before" << std::endl;
    // Simulator sim;
    // std::cout << sim.wind_spd() << std::endl;
    // Vector2_d start(0, 0);
    // double cost = sim.simulate_waypoints(start, start + Vector2_d(best[0], best[1]));
    // std::cout << "yaw: " << sim.path_bearing() << std::endl;
    // std::cout << "xtrack: " << sim.xtrack_error() << std::endl;
    // std::cout << "pos: " << sim.pos() << std::endl;
    // std::cout << "cost: " << cost << std::endl;

    // double neg_wind_dir = to_rad(-Constants::wind_dir());
    // Vector2_d estimated_goal = sim.pos().x()*Vector2_d(std::cos(neg_wind_dir), std::sin(neg_wind_dir));
    // std::cout << "estimated goal: " << estimated_goal << std::endl;
    // double estimate = wind_corrected_distance(
    //     Vector2_d(0, 0),
    //     estimated_goal
    // );
    // std::cout << "estimate: " << estimate << std::endl;

    // Vector2_f init = sim.simulate_const_yrate(atoi(argv[3]), 0.3841/4);
    // std::cout << "init x: " << init[0];
    // std::cout << "init y: " << init[1];

    Simulator sim;

    sim.reset(0, 0, 0, -45);
    std::vector<Vector2_d> traj;
    std::vector<Vector2_d> mission{
        Vector2_d(0, 0),
        Vector2_d(50, 150),
        Vector2_d(75, -100)
    };

    traj = sim.simulate_mission(mission);
    sim.save_trajectory(S2LatLng::FromDegrees(0, 0), traj, "../sim_roll_neg.txt");

    sim.reset(0, 0, 0, 45);
    traj = sim.simulate_mission(mission);
    sim.save_trajectory(S2LatLng::FromDegrees(0, 0), traj, "../sim_roll_pos.txt");
}