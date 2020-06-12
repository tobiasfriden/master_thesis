#ifndef _OPT_H_
#define _OPT_H_

#include "Evaluator.hpp"
#include "Mads.hpp"

#include "simulation.h"

void set_params(NOMAD::Parameters &p);

class SimEvaluator : public NOMAD::Evaluator {
public:
    SimEvaluator(const NOMAD::Parameters &p) : NOMAD::Evaluator(p) {};
    ~SimEvaluator() {};

    void set_simulator_params(double airspeed, double wind_spd, double wind_dir, double roll_max, double init_roll_) {
        sim = Simulator(airspeed, wind_spd, wind_dir, roll_max);
        init_roll = init_roll_;
    }

    void set_goal(double goal_hdg_, double hdg_error_=5, double xtrack_error_=5) {
        goal_hdg=goal_hdg_;
        hdg_error=hdg_error_;
        xtrack_error=xtrack_error_;
    }

    void start_logging(){
        log = true;
        out.open("../log_opt.txt");
    }

    void stop_logging(){
        log = false;
        out.close();
    }

    Vector3_d get_cost_and_error(double x_goal, double y_goal, double wind_scale){
        sim.set_wind(wind_scale*Constants::wind_spd(), sim.wind_dir());
        sim.reset(0, 0, 0, init_roll);
        Vector2_d start(0, 0);
        Vector2_d goal(x_goal, y_goal);
        double cost = sim.simulate_waypoints(
            start,
            goal
        );
        double xtrack_error = std::abs(sim.xtrack_error()) - Constants::xtrack_error();
        xtrack_error = std::max(xtrack_error, 0.0);
        cost += xtrack_error*Constants::xtrack_w();

        for(auto rate : sim.roll_rates()){
            cost += Constants::roll_rate_w()*std::pow(rate, 2)*sim.dt();
        }

        return Vector3_d(cost, sim.path_bearing(), sim.xtrack_error());        
    }

    double evaluate_pareto_point(double x_goal, double y_goal){
        Vector3_d low_vals = get_cost_and_error(x_goal, y_goal, 1 - Constants::wind_error());
        Vector3_d high_vals = get_cost_and_error(x_goal, y_goal, 1 + Constants::wind_error());
        return std::abs(low_vals[2]) + std::abs(high_vals[2]);       
    }

    Vector2_d get_best_pareto_point(NOMAD::Pareto_Front* front){
        double best_cost = std::numeric_limits<double>().max();
        const NOMAD::Eval_Point* best_point;
        auto point = front -> begin();

        while(point){
            double cost = evaluate_pareto_point((*point)[0].value(), (*point)[1].value());
            if(cost < best_cost){
                best_point = point;
                best_cost = cost;
            }
            point = front -> next();
        }
        return Vector2_d((*best_point)[0].value(), (*best_point)[1].value());
    }

    bool eval_x( NOMAD::Eval_Point &x, const NOMAD::Double &h_max, bool &count_eval) {
        Vector3_d low_vals = get_cost_and_error(x[0].value(), x[1].value(), 1 - Constants::wind_error());
        Vector3_d high_vals = get_cost_and_error(x[0].value(), x[1].value(), 1 + Constants::wind_error());
        
        double low_hdg_error = fabs(low_vals[1] - goal_hdg);
        low_hdg_error = fmin(low_hdg_error, 360-low_hdg_error);
        double high_hdg_error = fabs(high_vals[1] - goal_hdg);
        high_hdg_error = fmin(high_hdg_error, 360-high_hdg_error);

        // x.set_bb_output(0, low_vals[0]);
        // x.set_bb_output(1, high_vals[0]);
        // x.set_bb_output(2, low_hdg_error-hdg_error);
        // x.set_bb_output(3, high_hdg_error-hdg_error);
        x.set_bb_output(0, std::max(low_vals[0], high_vals[0]));
        x.set_bb_output(1, std::max(low_hdg_error, high_hdg_error) - hdg_error);
        if(log){
            out << x.value(0) << " " << x.value(1) << std::endl;
        }

        count_eval = true;
        return true;
    }
    Simulator sim;

private:
    double goal_hdg{0};
    double hdg_error{0};
    double xtrack_error{0};
    double init_roll{0};

    bool log{false};
    std::ofstream out;
};
#endif