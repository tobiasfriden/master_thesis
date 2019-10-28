#ifndef CONSTANTS_H_
#define CONSTANTS_H_

namespace Constants{
    double read_env(std::string env){
        return atof(std::getenv(env.c_str()));
    }

    // Simulation parameters
    double wind_dir(){
        return read_env("WIND_DIR");
    }
    double wind_spd(){
        return read_env("WIND_SPD");
    }
    double airspeed(){
        return read_env("AIRSPEED");
    }
    double yrate(){
        return read_env("YRATE");
    }

    // Optimization constraints
    double xtrack_error(){
        return read_env("XTRACK_ERROR");
    }
    double hdg_error(){
        return read_env("HDG_ERROR");
    }

    // Primitives
    static const int prim_hdg_size = 10;
    static const int prim_headings = 16;

    // Grid definition
    static const double cell_size = 10;
    static const double hdg_size = 20;
    static const double goal_size = 25;

    // HLUT definition
    static const double hlut_size = 40;
    static const double hlut_hdg = 20;
    double hlut_inner(){
        return read_env("HLUT_INNER");
    }
    double hlut_outer(){
        return read_env("HLUT_OUTER");
    }

    // Obstacles
    double safety_dist(){
        return read_env("SAFETY_DIST");
    }
    double start_offset(){
        return read_env("START_OFFSET");
    }

    int heading_index(double heading, double interval){
        int idx = int(std::floor(heading/interval));
        int next_idx = idx + 1;
        double diff = std::abs(heading - idx*interval);
        double next_diff = std::abs(heading - next_idx*interval);
        int best_idx = diff < next_diff ? idx : next_idx;
        if(best_idx >= 360/interval) best_idx -= 360/interval;
        return best_idx;
    }
};

#endif