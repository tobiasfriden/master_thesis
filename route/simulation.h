#ifndef SIMULATION_H_
#define SIMULATION_H_

#include <algorithm>

#include <mapbox/geojson.hpp>
#include <mapbox/geojson_impl.hpp>
#include <mapbox/geometry.hpp>
#include <s2/s2latlng.h>
#include <s2/util/math/vector.h>
#include "Evaluator.hpp"

#include "geo_utils.h"
#include "constants.h"

class Simulator {
public:
    Simulator()
      : _airspeed{Constants::airspeed()},
        _wind_spd{Constants::wind_spd()},
        _wind_dir{Constants::wind_dir()},
        _wind_error{Constants::wind_error()},
        _yrate_max{Constants::yrate()},
        _pos{Vector2_d(0, 0)} {};

    Simulator(
        double airspeed,
        double wind_spd,
        double wind_dir,
        double yrate_max
    ) : _airspeed{airspeed},
        _wind_spd{wind_spd},
        _wind_dir{wind_dir},
        _yrate_max{yrate_max}
    {
        _pos = Vector2_d(0, 0);
    };

    void reset(double x, double y, double yaw);
    void set_wind(double wind_spd, double wind_dir);

    double simulate_waypoints(
        Vector2_d const& prev_wp,
        Vector2_d const& next_wp,
        bool correct_wind=true
    );

    double simulate_waypoints(
        Vector2_d const& prev_wp,
        Vector2_d const& next_wp,
        std::vector<Vector2_d>& trajectory,
        bool correct_wind=true
    );
    std::vector<Vector2_d> simulate_mission(std::vector<Vector2_d> mission);
    void save_trajectory(S2LatLng const& origin, std::vector<Vector2_d> const& traj, std::string path);

    Vector2_d pos() const { return _pos; };
    double yaw() const { return _yaw; };
    double path_bearing() const { return _path_bearing; };
    double xtrack_error() const { return _xtrack_error; };
    double hdg_diff() const { return _hdg_diff; };
    double wind_spd() const { return _wind_spd; };
    double wind_dir() const { return _wind_dir; };
    double airspeed() const { return _airspeed; };
    double yrate_max() const { return _yrate_max; };

private:
    Vector2_d step(Vector2_d const& prev_wp, Vector2_d const& next_wp);

    double L1_acc(Vector2_d const& prev_wp, Vector2_d const& next_wp, Vector2_d groundspeed);
    Vector2_d groundspeed_vector();
    Vector2_d airspeed_vector();
    Vector2_d wind_vector();
    double wind_correction_angle();

    Vector2_d _pos;
    double _yaw{0};

    double _airspeed{0.0};
    double _wind_spd{0.0};
    double _wind_dir{0.0};
    double _wind_error{0.0};
    double _yrate_max{0.0};
    double _dt{0.25};

    double _L1_period{16.0};
    double _L1_damping{0.75};

    double _path_bearing{0};
    double _hdg_diff{0};
    double _xtrack_error{0};
};

#endif