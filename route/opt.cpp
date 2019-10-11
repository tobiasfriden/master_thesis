#include "opt.h"

void set_params(NOMAD::Parameters &p){
    p.set_DIMENSION(2);

    std::vector<NOMAD::bb_output_type> bbot(3);
    bbot[0] = NOMAD::OBJ;
    bbot[1] = NOMAD::EB;
    bbot[2] = NOMAD::EB;
    p.set_BB_OUTPUT_TYPE(bbot);

    // NOMAD::Point x0(2);
    // x0[0] = 50;
    // x0[1] = 50;
    // p.set_X0(x0);

    NOMAD::Point lb(2);
    lb[0] = -1000;
    lb[1] = 0;
    p.set_LOWER_BOUND(lb);
    p.set_UPPER_BOUND(NOMAD::Point(2, 1000.0));

    p.set_MAX_BB_EVAL(500);
    p.set_MIN_MESH_SIZE(NOMAD::Point(2, 0.5));
}

int main(int argc, char** argv) {
    NOMAD::Display out(std::cout);
    NOMAD::Parameters p(out);
    set_params(p);

    SimEvaluator se(p);
    NOMAD::Mads mads(p, &se);

    se.set_simulator_params(14, 7.5, 0, 0.3);
    se.set_goal(180, 10, 5);
    auto res = mads.run();
    std::cout << se.sim.path_bearing() << std::endl;
    std::cout << se.sim.xtrack_error() << std::endl;

    Simulator sim(14, 7.5, 0, 0.3);
    auto start = S2LatLng::FromDegrees(0, 0);
    sim.simulate_waypoints(start, offset(start, -120, 1));
    std::cout << std::endl;
    std::cout << sim.path_bearing() << std::endl;
    std::cout << sim.xtrack_error() << std::endl;
    std::cout << get_distance_NE(start, sim.pos()) << std::endl;
}