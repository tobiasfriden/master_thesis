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

    void set_simulator_params(float airspeed, float wind_spd, float wind_dir, float yrate_max) {
        sim = Simulator(airspeed, wind_spd, wind_dir, yrate_max);
    }

    void set_goal(float goal_hdg_, float hdg_error_=5, float xtrack_error_=5) {
        goal_hdg=goal_hdg_;
        hdg_error=hdg_error_;
        xtrack_error=xtrack_error_;
    }

    bool eval_x( NOMAD::Eval_Point &x, const NOMAD::Double &h_max, bool &count_eval) {
        sim.reset(0, 0, 0);
        S2LatLng start = S2LatLng::FromDegrees(0, 0);
        float cost = sim.simulate_waypoints(
            start,
            offset(start, x[0].value(), x[1].value())
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
    float goal_hdg{0};
    float hdg_error{0};
    float xtrack_error{0};
};
#endif