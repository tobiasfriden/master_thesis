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
    double wind_error(){
        return read_env("WIND_ERROR");
    }
    double airspeed(){
        return read_env("AIRSPEED");
    }
    double yrate(){
        return read_env("YRATE");
    }

    // Optimization
    double xtrack_error(){
        return read_env("XTRACK_ERROR");
    }
    double xtrack_w(){
        return read_env("XTRACK_W");
    }
    double hdg_error(){
        return read_env("HDG_ERROR");
    }
    double hdg_w(){
        return read_env("HDG_W");
    }

    // Primitives
    static const int prim_hdg_size = 20;
    static const int prim_headings = 9;
    static const int prim_headings_safe = 8;

    // Grid definition
    static const double cell_size = 10;
    static const double hdg_size = 20;
    static const double goal_size = 25;

    // HLUT definition
    static const double hlut_size = 10;
    static const double hlut_hdg = 20;
    double hlut_inner(){
        return read_env("HLUT_INNER");
    }
    double hlut_outer(){
        return read_env("HLUT_OUTER");
    }
    double hlut_entries(){
        return read_env("HLUT_ENTRIES");
    }

    // Obstacles
    double safety_dist(){
        return read_env("SAFETY_DIST");
    }
    double start_offset(){
        return read_env("START_OFFSET");
    }

    // Landing
    double flare_sink(){
        return read_env("FLARE_SINK");
    }
    double flare_sink_real(){
        return read_env("FLARE_SINK_REAL");
    }
    double flare_alt(){
        return read_env("FLARE_ALT");
    }
    double flare_sec(){
        return read_env("FLARE_SEC");
    }
    double max_sink(){
        return read_env("MAX_SINK");
    }
    double safety_alt(){
        return read_env("SAFETY_ALT");
    }
    double start_alt(){
        return read_env("START_ALT");
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