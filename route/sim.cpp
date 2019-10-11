#include <iostream>

#include <s2/s2latlng.h>

#include "Evaluator.hpp"

#include "simulation.h"


Vector2_i grid_search(int num_cells, int step, float wind_dir, float goal_hdg){
    int best_x = step;
    int best_y = step;
    float cost;
    float best_cost = std::numeric_limits<float>().max();
    float best_yaw = 0;

    S2LatLng start = S2LatLng::FromDegrees(0, 0);
    S2LatLng goal;

    Simulator sim(14, 7.5, wind_dir, 0.3);
    for(int i=-num_cells; i<=num_cells; i++){
        for(int j=1; j<=num_cells; j++){
            sim.reset(0, 0, 0);
            goal = offset(start, i*step, j*step);
            cost = sim.simulate_waypoints(start, goal);
            if(std::fabs(sim.path_bearing() - goal_hdg) < 10 &&
               std::fabs(sim.xtrack_error()) < 10 &&
               cost < best_cost){
                std::cout << "found better: x: " << step*i << " y: " << step*j << " brng: " << sim.path_bearing() << std::endl;
                std::cout << sim.pos().ToStringInDegrees() << std::endl;
                best_cost = cost;
                best_x = step*i;
                best_y = step*j;
                best_yaw = sim.path_bearing();
            }
            
        }
    }
    std::cout << "best x: " << best_x << std::endl;
    std::cout << "best y: " << best_y << std::endl;
    std::cout << "best yaw: " << best_yaw << std::endl;
    std::cout << "best cost: " << best_cost << std::endl;
    return Vector2_i(best_x, best_y);
}

int main(int argc, char** argv){
    int num_of_cells = atoi(argv[1]);
    int step = atoi(argv[2]);
    int wind_dir = atof(argv[3]);
    int goal_hdg = atof(argv[4]);
    Vector2_i best = grid_search(num_of_cells, step, wind_dir, goal_hdg);
    std::cout << best << std::endl;
    Simulator sim(14, 7.5, 0, 0.3);
    S2LatLng start = S2LatLng::FromDegrees(0, 0);
    sim.simulate_waypoints(start, offset(start, best[0], best[1]));
    std::cout << "yaw: " << sim.path_bearing() << std::endl;
    std::cout << "xtrack: " << sim.xtrack_error() << std::endl;

    // Vector2_f init = sim.simulate_const_yrate(atoi(argv[3]), 0.3841/4);
    // std::cout << "init x: " << init[0];
    // std::cout << "init y: " << init[1];
}