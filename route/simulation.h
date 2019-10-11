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

class Simulator {
public:
    Simulator() {};

    Simulator(
        float airspeed,
        float wind_spd,
        float wind_dir,
        float yrate_max
    ) : _airspeed{airspeed},
        _wind_spd{wind_spd},
        _wind_dir{wind_dir},
        _yrate_max{yrate_max}
    {
        _pos = S2LatLng::FromDegrees(0, 0);
    };

    void reset(float lat, float lng, float yaw);

    float simulate_waypoints(
        S2LatLng const& prev_wp,
        S2LatLng const& next_wp,
        bool correct_wind=true
    );

    float simulate_waypoints(
        S2LatLng const& prev_wp,
        S2LatLng const& next_wp,
        std::vector<S2LatLng>& trajectory,
        bool correct_wind=true
    );
    std::vector<S2LatLng> simulate_mission(std::vector<S2LatLng> mission);
    void save_trajectory(std::vector<S2LatLng> traj, std::string path);
    Vector2_f simulate_const_yrate(float goal_hdg, float yrate);

    S2LatLng pos() const { return _pos; };
    float yaw() const { return _yaw; };
    float path_bearing() const { return _path_bearing; };
    float xtrack_error() const { return _xtrack_error; };
    float hdg_diff() const { return _hdg_diff; };
    float wind_spd() const { return _wind_spd; };
    float wind_dir() const { return _wind_dir; };

private:
    Vector2_f step(S2LatLng const& prev_wp, S2LatLng const& next_wp);
    void step_const_yrate(float yrate);

    float L1_acc(S2LatLng const& prev_wp, S2LatLng const& next_wp, Vector2_f groundspeed);
    Vector2_f groundspeed_vector();
    float wind_correction_angle();

    S2LatLng _pos;
    float _yaw{0};

    float _airspeed{0.0};
    float _wind_spd{0.0};
    float _wind_dir{0.0};
    float _yrate_max{0.0};
    float _dt{0.1};

    float _L1_period{16.0};
    float _L1_damping{0.75};

    float _path_bearing{0};
    float _hdg_diff{0};
    float _xtrack_error{0};
};

#endif