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

    void set_simulator_params(double airspeed, double wind_spd, double wind_dir, double yrate_max) {
        sim = Simulator(airspeed, wind_spd, wind_dir, yrate_max);
    }

    void set_goal(double goal_hdg_, double hdg_error_=5, double xtrack_error_=5) {
        goal_hdg=goal_hdg_;
        hdg_error=hdg_error_;
        xtrack_error=xtrack_error_;
    }

    bool eval_x( NOMAD::Eval_Point &x, const NOMAD::Double &h_max, bool &count_eval) {
        sim.reset(0, 0, 0);
        Vector2_d start(0, 0);
        Vector2_d goal(x[0].value(), x[1].value());
        double cost = sim.simulate_waypoints(
            start,
            goal
        );
        cost += fabs(sim.path_bearing() - goal_hdg)*10;
        cost += sim.xtrack_error()*10;
        x.set_bb_output(0, cost);
        x.set_bb_output(1, fabs(sim.path_bearing()-goal_hdg)-hdg_error);
        x.set_bb_output(2, fabs(sim.xtrack_error())-xtrack_error);

        count_eval = true;
        return true;
    }
    Simulator sim;

private:
    double goal_hdg{0};
    double hdg_error{0};
    double xtrack_error{0};
};
#endif