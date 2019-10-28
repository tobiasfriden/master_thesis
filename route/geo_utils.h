#ifndef GEO_UTILS_H_
#define GEO_UTILS_H_

#include <math.h>
#include <s2/s2latlng.h>
#include <s2/util/math/vector.h>

#include "constants.h"

double longitude_scale(S2LatLng const& ll){
    double scale = cos(ll.lat());
    return fmax(scale, 0.01);
};

// NE distance between latlngs in meters
Vector2_d get_distance_NE(S2LatLng const& ll1, S2LatLng const& ll2){
    double scaling = 0.011131884502145034;
    return Vector2_d(
        (ll2.lat().e7() - ll1.lat().e7())*scaling,
        (ll2.lng().e7() - ll1.lng().e7())*scaling*longitude_scale(ll1)
    );
}

Vector2_d get_distance_NE(Vector2_d const& pos1, Vector2_d const& pos2) {
    return pos2 - pos1;
}


// check if point projected to line lies beyond line finish points
bool passed_point(S2LatLng const& point, S2LatLng const& ll1, S2LatLng const& ll2){
    Vector2_d pos_diff = get_distance_NE(ll1, ll2);
    Vector2_d point_diff = get_distance_NE(ll1, point);
    double dsquared = pos_diff.Norm2();
    return pos_diff.DotProd(point_diff)/dsquared >= 1.0;
}

// check if point projected to line lies beyond line finish points
bool passed_point(Vector2_d const& point, Vector2_d const& ll1, Vector2_d const& ll2){
    Vector2_d pos_diff = ll2 - ll1;
    Vector2_d point_diff = point - ll1;
    double dsquared = pos_diff.Norm2();
    return pos_diff.DotProd(point_diff)/dsquared >= 1.0;
}

// offset latlng meters in north and east direction
S2LatLng offset(S2LatLng const& ll, double d_north_m, double d_east_m){
    double scaling = 1/0.011131884502145034;
    double d_lat = d_north_m*scaling;
    double d_lng = d_east_m*scaling/longitude_scale(ll);
    return S2LatLng::FromE7(
        ll.lat().e7()+d_lat,
        ll.lng().e7()+d_lng
    );
}

double to_deg(double radians){
    return (radians*180)/M_PI;
}

double to_rad(double degrees){
    return (degrees*M_PI)/180;
}

S2LatLng offset_bearing(S2LatLng const& ll, double bearing, double dist_m){
    double d_north = cosf(to_rad(bearing))*dist_m;
    double d_east = sinf(to_rad(bearing))*dist_m;
    return offset(ll, d_north, d_east);
}

Vector2_d offset_bearing(Vector2_d const&pos, double bearing, double dist_m){
    double d_north = cosf(to_rad(bearing))*dist_m;
    double d_east = sinf(to_rad(bearing))*dist_m;
    return Vector2_d(pos.x() + d_north, pos.y() + d_east);  
}

// bearing angle between two latlngs
double bearing_to(S2LatLng const& ll1, S2LatLng const& ll2){
    double off_east = ll2.lng().degrees() - ll1.lng().degrees();
    double off_north = (ll2.lat().degrees() - ll1.lat().degrees())/longitude_scale(ll2);
    double bearing = 90 + to_deg(atan2f(-off_north, off_east));
    if(bearing < 0) bearing += 360;
    return bearing;
}

double bearing_to(Vector2_d const& pos1, Vector2_d const& pos2){
    Vector2_d ofs = pos2 - pos1;
    double bearing = to_deg(atan2(ofs.y(), ofs.x()));
    if(bearing < 0) bearing += 360;
    return bearing;
}

double clamp(double value, double min, double max){
    if(value < min){
        return min;
    } else if(value > max){
        return max;
    }
    return value;
}

double wrap_heading_360(double hdg){
    if(hdg < 0) hdg += 360;
    return fmod(hdg, 360);
}

// Input in deg
Vector2_d rotate(const Vector2_d& vec, double angle){
    double x = vec.x(), y = vec.y();
    double angle_rad = to_rad(angle);
    return Vector2_d(
        x*cos(angle_rad) - y*sin(angle_rad),
        y*cos(angle_rad) + x*sin(angle_rad)
    );
}

double wind_scaling(bool upwind, double airspeed=Constants::airspeed(), double wind_spd=Constants::wind_spd()){
    if(!upwind){
        wind_spd = -wind_spd;
    }
    return airspeed/(airspeed + wind_spd);
}

// Input and output in degrees
double wind_correction_angle(
    double heading,
    double airspeed,
    double wind_dir,
    double wind_spd
){
    double hdg_rad = to_rad(heading);
    double wind_dir_rad = to_rad(wind_dir);
    double wca = -std::asin(wind_spd/airspeed*std::sin(wind_dir_rad-hdg_rad));
    wca += hdg_rad;
    return wrap_heading_360(to_deg(wca));
}

// Actual flewn distance between points in wind frame (wind along x-axis)
// Points are defined in inertial coordinates
double wind_corrected_distance(
    const Vector2_d& pos1,
    const Vector2_d& pos2,
    double airspeed=Constants::airspeed(),
    double wind_spd=Constants::wind_spd()
){
    double heading = bearing_to(pos1, pos2);
    double heading_rad = to_rad(heading);
    double wca = wind_correction_angle(heading, airspeed, 0, wind_spd);
    double wca_rad = to_rad(wca);

    // Total velocity vector
    Vector2_d v_tot_vec(airspeed*std::cos(wca_rad) + wind_spd, airspeed*std::sin(wca_rad));
    Vector2_d hdg_dir(std::cos(heading_rad), std::sin(heading_rad));
    
    // Velocity along wanted path
    double V = v_tot_vec.DotProd(hdg_dir);

    // Time to reach goal
    double t = (pos2-pos1).Norm()/V;
    return t*airspeed;
}



#endif