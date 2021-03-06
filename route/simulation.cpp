#include "simulation.h"

void Simulator::reset(double x, double y, double new_yaw, double roll){
    _pos = Vector2_d(x, y);
    _yaw = new_yaw;
    _yaw_rate = 0;
    _roll = roll;
    _roll_rate = 0;
    _lateral_spd = 0;
    _hdg_diff = 0;
    _path_bearing = 0;
    _roll_rates.clear();
    _rolls.clear();
    _roll_cmds.clear();
    _yaw_rates.clear();
}

void Simulator::set_wind(double wind_spd, double wind_dir){
    _wind_spd = wind_spd;
    _wind_dir = wind_dir;
}

// Simulate transition between two waypoints
double Simulator::simulate_waypoints(
    Vector2_d const& prev_wp,
    Vector2_d const& next_wp,
    std::vector<Vector2_d>& trajectory,
    bool correct_wind
){
    if(correct_wind){
        _yaw = wind_correction_angle();
    }
    double cost = 0;
    while(!passed_point(_pos, prev_wp, next_wp)){
        trajectory.push_back(_pos);
        if((next_wp-_pos).Norm() <= _wp_r){
            break;
        }
        cost += step(prev_wp, next_wp).Norm();
    }
    return cost;
}
// Simulate transition between two waypoints
double Simulator::simulate_waypoints(
    Vector2_d const& prev_wp,
    Vector2_d const& next_wp,
    bool correct_wind
){
    if(correct_wind){
        _yaw = wind_correction_angle();
    }
    double cost = 0;
    while(!passed_point(_pos, prev_wp, next_wp)){
        if((next_wp-_pos).Norm() <= _wp_r){
            break;
        }
        cost += step(prev_wp, next_wp).Norm();
    }
    return cost;
}

std::vector<Vector2_d> Simulator::simulate_mission(std::vector<Vector2_d> mission) {
    std::vector<Vector2_d> trajectory;
    auto it = mission.begin();
    Vector2_d prev_wp = *it;
    it++;
    double cost = 0;
    while(it != mission.end()) {
        Vector2_d next_wp = *it;
        while(!passed_point(_pos, prev_wp, next_wp)){
            if((next_wp-_pos).Norm() <= _wp_r){
                break;
            }
            auto l = step(prev_wp, next_wp);
            cost += l.Norm();
            trajectory.push_back(_pos);
        }
        prev_wp = next_wp;
        it++;
    }
    std::cout << cost << std::endl;
    return trajectory;
}

void Simulator::save_trajectory(S2LatLng const& origin, std::vector<Vector2_d> const& trajectory, std::string path){
    std::ofstream out;
    mapbox::geojson::feature_collection fc;
    for(auto p : trajectory){
        mapbox::geojson::feature f;
        S2LatLng ll = offset(origin, p.x(), p.y());
        f.geometry = mapbox::geojson::point(ll.lng().degrees(), ll.lat().degrees());
        fc.push_back(f);
    }
    out.open(path.c_str());
    out << mapbox::geojson::stringify(fc);
    out.close();
}

void Simulator::save_rolls(std::string path){
    std::ofstream out;
    out.open(path.c_str());
    out << "TimeUS,RollRate,Roll,DesRoll" << std::endl;
    for(int i=0; i<_roll_rates.size(); i++){
        out << i*Constants::ts() << "," << _roll_rates[i] << "," << _rolls[i] << "," << _roll_cmds[i] << std::endl;
    }
    out.close();
}

void Simulator::update_roll(double roll_cmd){
    roll_cmd = clamp(roll_cmd, -_roll_max, _roll_max);
    double roll_new = _roll + _dt*_roll_rate;
    double roll_rate_new = _roll_rate -_dt/std::pow(_tau_s, 2)*_roll - 2*_xi*_dt/_tau_s*_roll_rate + _dt/std::pow(_tau_s, 2)*roll_cmd;
    _roll = roll_new;
    _roll_rate = roll_rate_new;

    double beta = std::atan(_lateral_spd/_airspeed);
    double V_tau = std::pow(_airspeed, 2) + std::pow(_lateral_spd, 2);

    _lateral_spd += _dt*(9.82*std::sin(to_rad(_roll)) - 1/std::cos(to_rad(_roll))*to_rad(_yaw_rate)*_airspeed + _roll_k*V_tau*beta*std::cos(beta));
    _roll_rates.push_back(_roll_rate);
    _rolls.push_back(_roll);
    _roll_cmds.push_back(roll_cmd);
}

// Step simulation, update position/_yaw and return air relative derivative
Vector2_d Simulator::step(Vector2_d const& prev_wp, Vector2_d const& next_wp){
    Vector2_d groundspeed = groundspeed_vector();
    Vector2_d pos_diff = groundspeed*_dt;

    double lat_acc = L1_acc(prev_wp, next_wp, groundspeed);
    if(use_roll_dynamics){
        double roll_cmd = to_deg(std::atan(lat_acc/9.82));
        update_roll(roll_cmd);
        _yaw_rate = to_deg(9.82/_airspeed*std::tan(to_rad(_roll)));
        _yaw_rates.push_back(_yaw_rate);
    } else {
        double yrate_max = 9.82/_airspeed*std::tan(to_rad(_roll_max));
        _yaw_rate = to_deg(clamp(lat_acc/_airspeed, yrate_max, -yrate_max));
    }

    double y_diff = _yaw_rate*_dt;

    _pos += pos_diff;
    _yaw += y_diff;
    _yaw = wrap_heading_360(_yaw);
    _hdg_diff += y_diff;
    _path_bearing = to_deg(std::atan2(groundspeed.y(), groundspeed.x()));
    _path_bearing = wrap_heading_360(_path_bearing);

    return _dt*airspeed_vector();
}


// Calculate demanded lateral acceleration
double Simulator::L1_acc(Vector2_d const& prev_wp, Vector2_d const& next_wp, Vector2_d groundspeed){
    double K_L1 = 4*_L1_damping*_L1_damping;
    double L1_dist = fmax(1/M_PI*_L1_damping*_L1_period*_airspeed, 0.1);

    Vector2_d ab = get_distance_NE(prev_wp, next_wp);
    if(ab.Norm() < 1e-6){
        ab = get_distance_NE(_pos, next_wp);
    }
    Vector2_d ab_unit = ab.Normalize();
    
    Vector2_d a_pos = get_distance_NE(prev_wp, _pos);
    // Save for reporting
    _xtrack_error = a_pos.CrossProd(ab_unit);
    double wp_a_dist = a_pos.Norm();

    double along_track_dist = a_pos.DotProd(ab_unit);
    double Nu = 0;
    double xtrack_v = 0;
    double ltrack_v = 0;
    if(wp_a_dist > L1_dist && along_track_dist/fmax(wp_a_dist, 1) < -0.7071){
        Vector2_d a_pos_unit = a_pos.Normalize();
        xtrack_v = groundspeed.CrossProd(-1*a_pos_unit);
        ltrack_v = groundspeed.DotProd(-1*a_pos_unit);
        Nu = atan2f(xtrack_v, ltrack_v);
    } else if(along_track_dist > (ab.Norm() + 3*_airspeed)){
        Vector2_d b_pos = get_distance_NE(next_wp, _pos);
        Vector2_d b_pos_unit = b_pos.Normalize();
        xtrack_v = groundspeed.CrossProd(-1*b_pos_unit);
        ltrack_v = groundspeed.DotProd(-1*b_pos_unit);
        Nu = atan2f(xtrack_v, ltrack_v);
    } else {
        xtrack_v = groundspeed.CrossProd(ab_unit);
        ltrack_v = groundspeed.DotProd(ab_unit);
        double Nu2 = atan2f(xtrack_v, ltrack_v);

        double sin_Nu1 = _xtrack_error/fmax(L1_dist, 0.1);
        sin_Nu1 = clamp(sin_Nu1, -0.7071, 0.7071);
        double Nu1 = asinf(sin_Nu1);

        Nu = Nu1 + Nu2;
    }

    Nu = clamp(Nu, -M_PI/2, M_PI/2);
    return K_L1*_airspeed*_airspeed/L1_dist*sinf(Nu);
}

// Calculate airspeed vector
Vector2_d Simulator::airspeed_vector() {
    Vector2_d airspeed_vec(cosf(to_rad(_yaw)), sinf(to_rad(_yaw)));
    Vector2_d lateral_vec(-sinf(to_rad(_yaw)), cosf(to_rad(_yaw)));
    return airspeed_vec*_airspeed + lateral_vec*_lateral_spd;
}

// Calculate wind compensated groundspeed vector
Vector2_d Simulator::groundspeed_vector() {
    return airspeed_vector() + wind_vector();
}

// Calculate wind vector
Vector2_d Simulator::wind_vector(){
    double wind_dir_rad = to_rad(_wind_dir);
    double yaw_rad = to_rad(_yaw);
    double err_factor = _wind_error*cosf(wind_dir_rad - yaw_rad);
    double scale = _wind_spd*(1 + err_factor);
    return scale*Vector2_d(cosf(wind_dir_rad), sinf(wind_dir_rad));
}

// Wind correction angle assuming convergence to path _yaw=0
double Simulator::wind_correction_angle() {
    return to_deg(-asinf(_wind_spd/_airspeed*sinf(to_rad(_wind_dir))));
}