#include <mapbox/geojson.hpp>
#include <mapbox/geojson_impl.hpp>
#include <mapbox/geometry.hpp>
#include "opt.h"

Vector2_i grid_search(SimEvaluator& se, int num_cells, double step, float wind_dir, float goal_hdg){
    int best_x = step;
    int best_y = step;
    float cost;
    float best_cost = std::numeric_limits<float>().max();
    float best_yaw = 0;
    Vector2_d start(0, 0);
    Vector2_d goal;
    Simulator sim;
    std::ofstream out("../grid_search.txt");
    for(int i=-num_cells; i<=num_cells; i++){
        for(int j=1; j<=num_cells; j++){
            goal = Vector2_d(i*step, j*step);
            // Vector3_d low_vals = se.get_cost_and_error(goal.x(), goal.y(), 1 - Constants::wind_error());
            // Vector3_d high_vals = se.get_cost_and_error(goal.x(), goal.y(), 1 + Constants::wind_error());
            Vector3_d mid = se.get_cost_and_error(goal.x(), goal.y(), 1);
            
            // double low_hdg_error = fabs(low_vals[1] - goal_hdg);
            // low_hdg_error = fmin(low_hdg_error, 360-low_hdg_error);
            // double high_hdg_error = fabs(high_vals[1] - goal_hdg);
            // high_hdg_error = fmin(high_hdg_error, 360-high_hdg_error);
            
            // double xtrack_error = std::abs(sim.xtrack_error()) - Constants::xtrack_error();
            // xtrack_error = std::max(xtrack_error, 0.0);
            // cost += xtrack_error*Constants::xtrack_w();
            
            //double hdg_error = std::max(low_hdg_error, high_hdg_error);
            double hdg_error = fabs(mid[1] - goal_hdg);
            cost = mid[0];
            if(hdg_error < Constants::hdg_error() && cost < 188){
                std::cout << Vector2_i(i, j) << std::endl;
                std::cout << step << std::endl;
                std::cout << cost << std::endl;
                std::cout << goal << std::endl;
            }
            if(hdg_error < Constants::hdg_error()){
                out << cost << " ";
            } else {
                out << -1 << " ";
            }
            if(hdg_error < Constants::hdg_error() &&
               cost < best_cost){
                best_cost = cost;
                best_x = step*i;
                best_y = step*j;
                best_yaw = sim.yaw();
            }
        }
        out << std::endl;
    }
    std::cout << "best cost: " << best_cost << std::endl;
    return Vector2_i(best_x, best_y);
}

int main(int argc, char** argv) {
    NOMAD::Display out(std::cout);
    NOMAD::Parameters p(out);
    set_params(p);
    p.set_X0(NOMAD::Point(2, 0));
    p.check();

    int wind_dir = atoi(argv[1]);
    int goal_hdg = atoi(argv[2]);

    SimEvaluator se(p);
    se.set_simulator_params(
        Constants::airspeed(),
        Constants::wind_spd(),
        wind_dir,
        Constants::yrate()
    );
    NOMAD::Mads mads(p, &se);

    NOMAD::Point x0(2);
    Vector2_i offset = grid_search(se, 160, 2.5, wind_dir, goal_hdg);
    std::cout << "offset: " << offset << std::endl;
    if(offset == Vector2_i(10, 10)){
        std::cout << "grid search failed: " << wind_dir << " " << goal_hdg << std::endl;
    }
    x0[0] = offset[0];
    x0[1] = fmax(offset[1], 0);
    p.set_X0(x0);
    p.check();

    std::cout << "ws: " << se.sim.wind_spd() << std::endl;
    std::cout << "wd: " << se.sim.wind_dir() << std::endl;
    se.set_goal(goal_hdg, Constants::hdg_error(), Constants::xtrack_error());
    se.start_logging();
    mads.run();
    se.stop_logging();
    auto sol = mads.get_best_feasible();
    std::cout << *sol << std::endl;

    Vector3_d low_vals = se.get_cost_and_error(sol -> value(0), sol -> value(1), 1 - Constants::wind_error());
    Vector3_d high_vals = se.get_cost_and_error(sol -> value(0), sol -> value(1), 1 + Constants::wind_error());
    Vector3_d mid = se.get_cost_and_error(sol -> value(0), sol -> value(1), 1);
    std::cout << "low: " << low_vals << std::endl;
    std::cout << "high: " << high_vals << std::endl;
    std::cout << "mid: " << mid << std::endl;

    Simulator sim;
    std::vector<Vector2_d> traj;
    sim.reset(0, 0, 0);
    sim.simulate_waypoints(
        Vector2_d(0, 0),
        Vector2_d(atof(argv[3]), atof(argv[4])),
        traj
    );
    sim.save_trajectory(S2LatLng::FromDegrees(0, 0), traj, "../opt_test_traj.txt");
    std::cout << sim.yaw() << std::endl;
    std::cout << se.get_cost_and_error(atof(argv[3]), atof(argv[4]), 1) << std::endl;
}