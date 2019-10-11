#include "simulation.h"

void Simulator::reset(float lat, float lng, float new_yaw){
    _pos = S2LatLng::FromDegrees(lat, lng);
    _yaw = new_yaw;
    _hdg_diff = 0;
    _path_bearing = 0;
}

// Simulate transition between two waypoints
float Simulator::simulate_waypoints(
    S2LatLng const& prev_wp,
    S2LatLng const& next_wp,
    std::vector<S2LatLng>& trajectory,
    bool correct_wind
){
    if(correct_wind){
        _yaw = wind_correction_angle();
    }
    float cost = 0;
    while(!passed_point(_pos, prev_wp, next_wp)){
        trajectory.push_back(_pos);
        if(get_distance_NE(_pos, next_wp).Norm() < 10){
            break;
        }
        cost += 1 + step(prev_wp, next_wp).Norm();
    }
    return cost;
}
// Simulate transition between two waypoints
float Simulator::simulate_waypoints(
    S2LatLng const& prev_wp,
    S2LatLng const& next_wp,
    bool correct_wind
){
    if(correct_wind){
        _yaw = wind_correction_angle();
    }
    float cost = 0;
    while(!passed_point(_pos, prev_wp, next_wp)){
        if(get_distance_NE(_pos, next_wp).Norm() < 10){
            break;
        }
        cost += 1 + step(prev_wp, next_wp).Norm();
    }
    return cost;
}

std::vector<S2LatLng> Simulator::simulate_mission(std::vector<S2LatLng> mission) {
    std::vector<S2LatLng> trajectory;
    reset(mission[0].lat().degrees(), mission[0].lng().degrees(), 0);
    auto it = mission.begin();
    S2LatLng prev_wp = *it;
    it++;
    while(it != mission.end()) {
        S2LatLng next_wp = *it;
        while(!passed_point(_pos, prev_wp, next_wp)){
            if(get_distance_NE(_pos, next_wp).Norm() < 1){
                break;
            }
            step(prev_wp, next_wp);
            trajectory.push_back(_pos);
        }
        prev_wp = next_wp;
        it++;
    }
    return trajectory;
}

void Simulator::save_trajectory(std::vector<S2LatLng> trajectory, std::string path){
    std::ofstream out;
    mapbox::geojson::feature_collection fc;
    for(auto ll : trajectory){
        mapbox::geojson::feature f;
        f.geometry = mapbox::geojson::point(ll.lng().degrees(), ll.lat().degrees());
        fc.push_back(f);
    }
    out.open(path.c_str());
    out << mapbox::geojson::stringify(fc);
    out.close();
}

Vector2_f Simulator::simulate_const_yrate(float goal_hdg, float yrate){
    _yaw = wind_correction_angle();
    float time = 0.0;
    std::cout << goal_hdg << std::endl;
    std::cout << _path_bearing << std::endl;
    while(fabs(goal_hdg-_path_bearing)>2.5 && time < 30){
        step_const_yrate(yrate);
        time += _dt;
    }
    return get_distance_NE(S2LatLng::FromDegrees(0, 0), _pos);
}

// Step simulation, update position/_yaw and return derivatives
Vector2_f Simulator::step(S2LatLng const& prev_wp, S2LatLng const& next_wp){
    Vector2_f groundspeed = groundspeed_vector();
    Vector2_f pos_diff = groundspeed*_dt;

    float lat_acc = L1_acc(prev_wp, next_wp, groundspeed);
    float yrate = clamp(lat_acc/_airspeed, -_yrate_max, _yrate_max);
    float y_diff = to_deg(yrate*_dt);

    S2LatLng last_pos = _pos;
    _pos = offset(_pos, pos_diff[0], pos_diff[1]);
    _yaw += y_diff;
    _yaw = fmod(_yaw, 360);
    _hdg_diff += y_diff;
    _path_bearing = bearing_to(last_pos, _pos);

    return Vector2_f(pos_diff[0], pos_diff[1]);
}

void Simulator::step_const_yrate(float yrate){
    Vector2_f groundspeed = groundspeed_vector();
    Vector2_f pos_diff = groundspeed*_dt;

    float y_diff = to_deg(yrate*_dt);

    S2LatLng last_pos = _pos;
    _pos = offset(_pos, pos_diff[0], pos_diff[1]);
    _yaw += y_diff;
    _path_bearing = bearing_to(last_pos, _pos);  
}


// Calculate demanded lateral acceleration
float Simulator::L1_acc(S2LatLng const& prev_wp, S2LatLng const& next_wp, Vector2_f groundspeed){
    float K_L1 = 4*_L1_damping*_L1_damping;
    float L1_dist = fmax(1/M_PI*_L1_damping*_L1_period*_airspeed, 0.1);

    Vector2_f ab = get_distance_NE(prev_wp, next_wp);
    if(ab.Norm() < 1e-6){
        ab = get_distance_NE(_pos, next_wp);
    }
    Vector2_f ab_unit = ab.Normalize();
    
    Vector2_f a_pos = get_distance_NE(prev_wp, _pos);
    // Save for reporting
    _xtrack_error = a_pos.CrossProd(ab_unit);
    float wp_a_dist = a_pos.Norm();

    float along_track_dist = a_pos.DotProd(ab_unit);
    float Nu = 0;
    float xtrack_v = 0;
    float ltrack_v = 0;
    if(wp_a_dist > L1_dist && along_track_dist/fmax(wp_a_dist, 1) < -0.7071){
        Vector2_f a_pos_unit = a_pos.Normalize();
        xtrack_v = groundspeed.CrossProd(-1*a_pos_unit);
        ltrack_v = groundspeed.DotProd(-1*a_pos_unit);
        Nu = atan2f(xtrack_v, ltrack_v);
    } else if(along_track_dist > (ab.Norm() + 3*_airspeed)){
        Vector2_f b_pos = get_distance_NE(next_wp, _pos);
        Vector2_f b_pos_unit = b_pos.Normalize();
        xtrack_v = groundspeed.CrossProd(-1*b_pos_unit);
        ltrack_v = groundspeed.DotProd(-1*b_pos_unit);
        Nu = atan2f(xtrack_v, ltrack_v);
    } else {
        xtrack_v = groundspeed.CrossProd(ab_unit);
        ltrack_v = groundspeed.DotProd(ab_unit);
        float Nu2 = atan2f(xtrack_v, ltrack_v);

        float sin_Nu1 = _xtrack_error/fmax(L1_dist, 0.1);
        sin_Nu1 = clamp(sin_Nu1, -0.7071, 0.7071);
        float Nu1 = asinf(sin_Nu1);

        Nu = Nu1 + Nu2;
    }

    Nu = clamp(Nu, -M_PI/2, M_PI/2);
    return K_L1*_airspeed*_airspeed/L1_dist*sinf(Nu);
}

// Calculate wind compensated groundspeed vector
Vector2_f Simulator::groundspeed_vector() {
    Vector2_f groundspeed(cosf(to_rad(_yaw)), sinf(to_rad(_yaw)));
    groundspeed *= _airspeed;

    Vector2_f wind_vec(cosf(to_rad(_wind_dir)), sinf(to_rad(_wind_dir)));
    wind_vec *= _wind_spd;
    return groundspeed + wind_vec;
}

// Wind correction angle assuming convergence to path _yaw=0
float Simulator::wind_correction_angle() {
    return to_deg(-asinf(_wind_spd/_airspeed*sinf(to_rad(_wind_dir))));
}