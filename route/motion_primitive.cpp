#include "motion_primitive.h"

Vector2_i grid_search(int num_cells, int step, float goal_hdg, float wind_spd, float wind_dir){
    int best_x = step;
    int best_y = step;
    float cost;
    float best_cost = std::numeric_limits<float>().max();
    float best_yaw = 0;

    S2LatLng start = S2LatLng::FromDegrees(0, 0);
    S2LatLng goal;

    Simulator sim(14, wind_spd, wind_dir, 0.3);
    for(int i=-num_cells; i<=num_cells; i++){
        for(int j=1; j<=num_cells; j++){
            sim.reset(0, 0, 0);
            goal = offset(start, i*step, j*step);
            float cost = sim.simulate_waypoints(start, goal);
            //cost += std::fabs(sim.path_bearing() - goal_hdg)*2.5;
            //cost += std::fabs(sim.xtrack_error())*10;
            if(std::fabs(sim.path_bearing() - goal_hdg) < 15 &&
               std::fabs(sim.xtrack_error()) < 5 &&
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
    for(int i=1; i<=18; i++){
        for(int j=0; j<16; j++){
            int goal_hdg = i*10;
            int wind_dir = int(std::rint(wind_directions[j]));
            
            Vector2_f offset(cosf(to_rad(goal_hdg)), sinf(to_rad(goal_hdg)));
            offset *= 100;
            NOMAD::Point x0(2);
            x0[0] = offset[0];
            x0[1] = fmax(offset[1], 0);
            std::cout << offset << std::endl;
            p.set_X0(x0);
            p.check();

            SimEvaluator se(p);
            NOMAD::Mads mads(p, &se);

            if(log) std::cout << "wind_dir: " << wind_dir << std::endl;

            se.set_simulator_params(14, wind_spd_, wind_dir, 0.3);
            se.set_goal(goal_hdg, 10, 5);
            auto res = mads.run();
            auto sol = mads.get_best_feasible();
            if(res == NOMAD::stop_type::DELTA_M_MIN_REACHED && sol){
                save(sol->value(0), sol->value(1), goal_hdg, wind_dir);
                if(log){
                    std::cout << "goal heading: " << goal_hdg << std::endl;
                    std::cout << "final bearing: " << se.sim.path_bearing() << std::endl;
                    std::cout << "final xtrack: " << se.sim.xtrack_error() << std::endl;
                }
            } else {
                std::cout << "failed: " << wind_dir << " " << goal_hdg << std::endl;
                Vector2_i best = grid_search(150, 1, goal_hdg, wind_spd_, wind_dir);
                if(best[0] > 1 || best[1] > 1){
                    std::cout << "found sol with gridsearch: " << best << std::endl;
                    save(best[0], best[1], goal_hdg, wind_dir);
                }
                fail++;
            }
        }
    }
    std::cout << "failed: " << fail << std::endl;
    std::cout << "primitives: " << mp_set.size() << std::endl;
}

void MotionPrimitiveSet::save(float north, float east, int goal_hdg, int wind_dir){
    MotionPrimitive mp(north, east, goal_hdg, wind_dir);
    mp_set.insert(mp);
}

void MotionPrimitiveSet::save_to_file(std::string output){
    std::ofstream out;
    out.open(output.c_str());
    for(auto it = mp_set.begin(); it!=mp_set.end(); it++){
        MotionPrimitive mp = *it;
        out << mp.north() << " " << mp.east() << " " << mp.goal_hdg() << " " << mp.wind_dir() << std::endl;
    }
    out.close();    
}

void MotionPrimitiveSet::load_from_file(std::string input){
    std::ifstream in;
    in.open(input.c_str());
    
    float north, east;
    int goal_hdg, wind_dir;
    std::string line;
    while(getline(in, line)){
        std::stringstream ss(line);
        ss >> north >> east >> goal_hdg >> wind_dir;
        save(north, east, goal_hdg, wind_dir);
    }
    in.close();
}

std::vector<Vector2_f> MotionPrimitiveSet::get_expansions(float heading, int wind_dir){
    std::vector<Vector2_f> expansions;
    float heading_diff = wind_dir - heading;
    int closest_wind = closest_wind_dir(heading_diff);
    int inverse_closest_wind = closest_wind_dir(360-heading_diff);

    // Forward primitive
    std::vector<int> forward_expansions{25, 50, 75, 100};
    for(auto i : forward_expansions){
        expansions.push_back(offset_vector(heading, i, 0));
    }

    MotionPrimitive mp(0, 0, 0, closest_wind);
    MotionPrimitive mp_inverse(0, 0, 0, inverse_closest_wind);
    for(int i=1; i<=18; i++){
        int goal_hdg = i*10;
        mp.set_goal_hdg(goal_hdg);
        mp_inverse.set_goal_hdg(goal_hdg);
        if(lookup(mp)){
            Vector2_f offset = offset_vector(heading, mp.north(), mp.east());
            expansions.push_back(offset);
        }
        if(lookup(mp_inverse)){
            Vector2_f offset = offset_vector(heading, mp_inverse.north(), -mp_inverse.east());
            expansions.push_back(offset);
        }
    }
    return expansions;
};

bool MotionPrimitiveSet::lookup(MotionPrimitive& mp){
    auto it = mp_set.find(mp);
    if(it == mp_set.end()){
        return false;
    }
    mp = *it;
    return true;
}

int MotionPrimitiveSet::closest_wind_dir(float heading_diff){
    if(heading_diff < 0){
        heading_diff += 360;
    }
    heading_diff = fmod(heading_diff, 360);
    float best_diff = 360;
    float best_dir = 0;
    float diff;
    for(int i=0; i<16; i++){
        diff = fabs(heading_diff - wind_directions[i]);
        if(diff < best_diff){
            best_diff = diff;
            best_dir = wind_directions[i];
        }
    }
    return int(std::rint(best_dir));
}

Vector2_f MotionPrimitiveSet::offset_vector(float offset_heading, float d_north, float d_east){
    S2LatLng offset = S2LatLng::FromDegrees(0, 0);
    offset = offset_bearing(offset, offset_heading, d_north);
    offset = offset_bearing(offset, offset_heading+90, d_east);
    return get_distance_NE(S2LatLng::FromDegrees(0, 0), offset);
}