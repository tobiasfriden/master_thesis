#include "motion_primitive.h"

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
            
            double xtrack_error = std::abs(sim.xtrack_error()) - Constants::xtrack_error();
            xtrack_error = std::max(xtrack_error, 0.0);
            cost += xtrack_error*Constants::xtrack_w();
            
            double hdg_error = std::abs(sim.path_bearing() - goal_hdg);
            hdg_error = std::min(hdg_error, std::abs(360-hdg_error));
            if(hdg_error < Constants::hdg_error() &&
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

void MotionPrimitiveSet::generate(bool log){
    std::ofstream os;
    NOMAD::Display out(os);
    NOMAD::Parameters p(out);
    set_params(p);

    int fail = 0;
    for(int i=1; i<=Constants::prim_headings; i++){
        for(int j=0; j<18; j++){
            int goal_hdg = i*Constants::prim_hdg_size;
            int wind_dir = j*20;
            
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

            if(log) std::cout << "wind_dir: " << wind_dir << std::endl;

            se.set_simulator_params(
                Constants::airspeed(),
                wind_spd_,
                wind_dir,
                Constants::yrate()
            );
            se.set_goal(goal_hdg, Constants::hdg_error(), Constants::xtrack_error());
            mads.run();
            auto sol = mads.get_best_feasible();
            //auto front = mads.get_pareto_front();
            if(sol){
                Vector2_d best_wp(sol -> value(0), sol -> value(1));
                se.sim.reset(0, 0, 0);
                double cost = se.sim.simulate_waypoints(Vector2_d(0, 0), best_wp);
                save(MotionPrimitive(
                    se.sim.pos().x(),
                    se.sim.pos().y(),
                    se.sim.path_bearing(),
                    best_wp.x(),
                    best_wp.y(),
                    cost,
                    goal_hdg,
                    wind_dir
                ));
                if(log){
                    std::cout << "goal heading: " << goal_hdg << std::endl;
                    std::cout << "final bearing: " << se.sim.path_bearing() << std::endl;
                    std::cout << "final xtrack: " << se.sim.xtrack_error() << std::endl;
                }
            } else {
                std::cout << "failed: " << wind_dir << " " << goal_hdg << std::endl;
                fail++;
            }
        }
    }
    std::cout << "failed: " << fail << std::endl;
    std::cout << "primitives: " << mp_set.size() << std::endl;
}

void MotionPrimitiveSet::save(const MotionPrimitive& mp){
    mp_set.insert(mp);
}

void MotionPrimitiveSet::save_to_file(std::string base_path){
    std::ostringstream full_path;
    full_path << base_path << int(wind_spd_) << ".txt"; 
    std::ofstream out;
    out.open(full_path.str().c_str());
    for(auto it = mp_set.begin(); it!=mp_set.end(); it++){
        MotionPrimitive mp = *it;
        out << mp;
    }
    out.close();    
}

void MotionPrimitiveSet::load_from_file(std::string base_path){
    std::ostringstream full_path;
    full_path << base_path << int(wind_spd_) << ".txt"; 
    std::ifstream in;
    in.open(full_path.str().c_str());
    
    MotionPrimitive mp;
    std::string line;
    while(getline(in, line)){
        std::stringstream ss(line);
        ss >> mp;
        mp_set.insert(mp);
    }
    in.close();
}

void MotionPrimitiveSet::save_visual(std::string path, double wind_dir, double wind_scale){
    Simulator sim(Constants::airspeed(), wind_scale*Constants::wind_spd(), wind_dir, Constants::yrate());

    std::ofstream out(path);

    std::vector<Vector2_d> traj;
    Vector2_d origin(0, 0);
    for(auto exp : get_expansions(0, wind_dir)){
        traj.clear();
        sim.reset(0, 0, 0);
        out << exp.x() << "," << exp.y() << " ";
        sim.simulate_waypoints(origin, exp, traj);
        for(auto point : traj){
            out << point.x() << "," << point.y() << " ";
        }
        out << std::endl;
    }
}

std::vector<Vector2_d> MotionPrimitiveSet::get_expansions(double heading, int wind_dir) const {
    std::vector<Vector2_d> expansions;
    double heading_diff = wind_dir - heading;
    int closest_wind = closest_wind_dir(heading_diff);
    int inverse_closest_wind = closest_wind_dir(360-heading_diff);

    heading_diff = std::min(heading_diff, std::abs(360-heading_diff));
    int prim_headings = Constants::prim_headings;
    if(std::abs(std::sin(to_rad(heading_diff))) > M_SQRT1_2){
        prim_headings = Constants::prim_headings_safe;
    }

    // Forward primitive
    std::vector<int> forward_expansions{25, 50, 75, 100};
    for(auto i : forward_expansions){
        expansions.push_back(offset_vector(heading, i, 0));
    }

    MotionPrimitive mp(closest_wind);
    MotionPrimitive mp_inverse(inverse_closest_wind);
    for(int i=1; i<=prim_headings; i++){
        int goal_hdg = i*Constants::prim_hdg_size;
        mp.set_goal_hdg(goal_hdg);
        mp_inverse.set_goal_hdg(goal_hdg);
        if(lookup(mp)){
            Vector2_d offset = offset_vector(heading, mp.wp_north(), mp.wp_east());
            expansions.push_back(offset);
        }
        if(lookup(mp_inverse)){
            Vector2_d offset = offset_vector(heading, mp_inverse.wp_north(), -mp_inverse.wp_east());
            expansions.push_back(offset);
        }
    }
    return expansions;
};

std::vector<MotionPrimitive> MotionPrimitiveSet::get_mp_expansions(double heading, int wind_dir) const {
    std::vector<MotionPrimitive> expansions;

    double heading_diff = wind_dir - heading;
    int closest_wind = closest_wind_dir(heading_diff);
    int inverse_closest_wind = closest_wind_dir(360-heading_diff);

    heading_diff = std::min(heading_diff, std::abs(360-heading_diff));
    int prim_headings = Constants::prim_headings;
    if(std::abs(std::sin(to_rad(heading_diff))) > M_SQRT1_2){
        prim_headings = Constants::prim_headings_safe;
    }

    std::vector<int> forward_expansions{10};
    Vector2_d origin(0, 0);
    for(auto i : forward_expansions){
        Vector2_d offset = offset_vector(heading, i, 0);
        double cost = wind_corrected_distance(
            origin,
            rotate(offset, -wind_dir)
        );
        expansions.push_back(
            MotionPrimitive(
                offset.x(),
                offset.y(),
                heading,
                offset.x(),
                offset.y(),
                cost,
                heading,
                wind_dir
            )
        );
    }

    MotionPrimitive mp(closest_wind);
    MotionPrimitive mp_inverse(inverse_closest_wind);

    for(int i=1; i<=prim_headings; i++){
        int goal_hdg = i*Constants::prim_hdg_size;
        mp.set_goal_hdg(goal_hdg);
        mp_inverse.set_goal_hdg(goal_hdg);
        Vector2_d offset_pos;
        Vector2_d offset_wp;
        if(lookup(mp)){
            offset_pos = offset_vector(heading, mp.north(), mp.east());
            offset_wp = offset_vector(heading, mp.wp_north(), mp.wp_east());
            expansions.push_back(
                MotionPrimitive(
                    offset_pos.x(),
                    offset_pos.y(),
                    heading + mp.heading(),
                    offset_wp.x(),
                    offset_wp.y(),
                    mp.cost(),
                    heading + goal_hdg,
                    wind_dir
                )
            );
        }
        if(lookup(mp_inverse)){
            offset_pos = offset_vector(heading, mp_inverse.north(), -mp_inverse.east());
            offset_wp = offset_vector(heading, mp_inverse.wp_north(), -mp_inverse.wp_east());
            expansions.push_back(
                MotionPrimitive(
                    offset_pos.x(),
                    offset_pos.y(),
                    heading - mp_inverse.heading(),
                    offset_wp.x(),
                    offset_wp.y(),
                    mp_inverse.cost(),
                    heading - goal_hdg,
                    wind_dir
                )
            );            
        }
    }
    return expansions;    
}

bool MotionPrimitiveSet::lookup(MotionPrimitive& mp) const {
    auto it = mp_set.find(mp);
    if(it == mp_set.end()){
        return false;
    }
    mp = *it;
    return true;
}

int MotionPrimitiveSet::closest_wind_dir(double heading_diff) const {
    heading_diff = wrap_heading_360(heading_diff);
    return Constants::heading_index(heading_diff, Constants::hdg_size)*Constants::hdg_size;
}

Vector2_d MotionPrimitiveSet::offset_vector(double offset_heading, double d_north, double d_east) const {
    Vector2_d offset(0, 0);
    offset = offset_bearing(offset, offset_heading, d_north);
    offset = offset_bearing(offset, offset_heading+90, d_east);
    return offset;
}