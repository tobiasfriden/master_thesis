#include "opt.h"

void set_params(NOMAD::Parameters &p){
    p.set_DIMENSION(2);

    std::vector<NOMAD::bb_output_type> bbot(2);
    bbot[0] = NOMAD::OBJ;
    bbot[1] = NOMAD::EB;
    // bbot[1] = NOMAD::OBJ;
    // bbot[2] = NOMAD::EB;
    // bbot[3] = NOMAD::EB;
    // bbot[4] = NOMAD::EB;
    // bbot[5] = NOMAD::EB;
    p.set_BB_OUTPUT_TYPE(bbot);

    NOMAD::Point lb(2);
    lb[0] = -1000;
    lb[1] = 0;
    p.set_LOWER_BOUND(lb);
    p.set_UPPER_BOUND(NOMAD::Point(2, 1000.0));

    p.set_MAX_BB_EVAL(500);
    //p.set_MULTI_NB_MADS_RUNS(100);
    p.set_MIN_MESH_SIZE(NOMAD::Point(2, 0.5));
    p.set_INITIAL_POLL_SIZE(50);
    p.set_DIRECTION_TYPE(NOMAD::ORTHO_2);

    //p.set_F_TARGET(NOMAD::Point(2, Constants::xtrack_w()));
}