#include <mapbox/geojson.hpp>
#include <mapbox/geojson_impl.hpp>
#include <mapbox/geometry.hpp>
#include "opt.h"

Vector2_i grid_search(int num_cells, int step, float wind_dir, float goal_hdg){
    int best_x = step;
    int best_y = step;
    float cost;
    float best_cost = std::numeric_limits<float>().max();
    float best_yaw = 0;
    Vector2_d start(0, 0);
    Vector2_d goal;
    Simulator sim;
    for(int i=-num_cells; i<=num_cells; i++){
        for(int j=1; j<=num_cells; j++){
            sim.reset(0, 0, 0);
            goal = Vector2_d(i*step, j*step);
            cost = sim.simulate_waypoints(start, goal);
            cost += fabs(sim.xtrack_error())*Constants::xtrack_w();
            double hdg_error = std::abs(sim.path_bearing() - goal_hdg);
            hdg_error = std::min(hdg_error, std::abs(360-hdg_error));
            if(hdg_error < Constants::hdg_error() &&
               std::abs(sim.xtrack_error()) < Constants::xtrack_error() &&
               cost < best_cost){
                best_cost = cost;
                best_x = step*i;
                best_y = step*j;
                best_yaw = sim.yaw();
            }
        }
    }
    return Vector2_i(best_x, best_y);
}

int main(int argc, char** argv) {
    NOMAD::Display out(std::cout);
    NOMAD::Parameters p(out);
    set_params(p);

    int wind_dir = atoi(argv[1]);
    int goal_hdg = atoi(argv[2]);

    NOMAD::Point x0(2);
    Vector2_i offset = grid_search(30, 10, wind_dir, goal_hdg);
    if(offset == Vector2_i(10, 10)){
        std::cout << "grid search failed: " << wind_dir << " " << goal_hdg << std::endl;
    }
    x0[0] = offset[0];
    x0[1] = fmax(offset[1], 0);
    p.set_X0(x0);
    p.check();

    SimEvaluator se(p);
    NOMAD::Mads mads(p, &se);

    se.set_simulator_params(
        Constants::airspeed(),
        Constants::wind_spd(),
        wind_dir,
        Constants::yrate()
    );
    se.set_goal(goal_hdg, Constants::hdg_error(), Constants::xtrack_error());
    auto res = mads.multi_run();
    std::cout << se.sim.path_bearing() << std::endl;
    std::cout << se.sim.xtrack_error() << std::endl;
    std::cout << res << std::endl;
    auto sol = mads.get_best_feasible();
    auto front = mads.get_pareto_front();
    auto best_point = se.get_best_pareto_point(front);
    std::cout << best_point << std::endl;
    std::cout << *sol << std::endl;

    Simulator sim(14, 7.5, 0, 0.3);
    Vector2_d start(0, 0);
    Vector2_d goal(-120, 1);
    sim.simulate_waypoints(start, goal);
    std::cout << std::endl;
    std::cout << sim.path_bearing() << std::endl;
    std::cout << sim.xtrack_error() << std::endl;
    std::cout << get_distance_NE(start, sim.pos()) << std::endl;
}