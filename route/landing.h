#ifndef LANDING_H_
#define LANDING_H_

#include <vector>
#include <s2/s2latlng.h>
#include <casadi/casadi.hpp>

#include "geo_utils.h"
#include "constants.h"
#include "obstacle.h"

class Landing{
public:
    Landing(
        Vector2_d origin,
        double width,
        double height,
        double rotation
    ) : _origin{origin},
        _width{width},
        _height{height},
        _rotation{rotation},
        _flare_sink{Constants::flare_sink()},
        _flare_sink_real{Constants::flare_sink_real()},
        _flare_sec{Constants::flare_sec()},
        _max_sink{Constants::max_sink()},
        _flare_h{Constants::flare_alt()},
        _wind_spd{Constants::wind_spd()},
        _wind_dir{Constants::wind_dir()},
        _airspeed{Constants::airspeed()} 
    {
        setup_area();
    };
    
    typedef std::vector<Vector2_d> line;

    bool feasible_heading(const Obstacles& obst, double hdg, double safety_h, double alt);
    double land_distance(double hdg, double alt);
    Vector2_d min_approach_point(double hdg, double alt);
    line optimize(double hdg, double safety_h=Constants::safety_alt(), double alt=Constants::start_alt());
    void save_to_file(std::string path);
    
private:
    void setup_area();

    bool is_height_collision(double hdg);
    line hdg_line(double hdg);
    std::vector<line> height_lines();
    std::vector<line> width_lines();
    Vector2_d intersection(line l1, line l2);
    double groundspeed(double hdg);
    line intersection_points(double hdg);


    // Area definition
    Vector2_d _origin;
    double _width;
    double _height;
    double _rotation;
    Vector2_d _center;
    Vector2_d _w_dir;
    Vector2_d _h_dir;
    Vector2_d _h_hdg_interval;
    bool _includes_360;

    // Wind
    double _wind_spd;
    double _wind_dir;
    double _airspeed;
    
    // Landing params
    double _flare_sink;
    double _flare_sink_real;
    double _max_sink;
    double _flare_h;
    double _flare_sec;
};


#endif