#ifndef GEO_UTILS_H_
#define GEO_UTILS_H_

#include <math.h>
#include <s2/s2latlng.h>
#include <s2/util/math/vector.h>

// def longitude_scale(Lat):
//     scale = np.cos(np.radians(Lat))
//     return np.max([scale, 0.01])
float longitude_scale(S2LatLng const& ll){
    float scale = cos(ll.lat());
    return fmax(scale, 0.01);
};

// NE distance between latlngs in meters
Vector2_f get_distance_NE(S2LatLng const& ll1, S2LatLng const& ll2){
    float scaling = 0.011131884502145034;
    return Vector2_f(
        (ll2.lat().e7() - ll1.lat().e7())*scaling,
        (ll2.lng().e7() - ll1.lng().e7())*scaling*longitude_scale(ll1)
    );
}


// check if point projected to line lies beyond line finish points
bool passed_point(S2LatLng const& point, S2LatLng const& ll1, S2LatLng const& ll2){
    Vector2_f pos_diff = get_distance_NE(ll1, ll2);
    Vector2_f point_diff = get_distance_NE(ll1, point);
    float dsquared = pos_diff.Norm2();
    return pos_diff.DotProd(point_diff)/dsquared >= 1.0;
}

// offset latlng meters in north and east direction
S2LatLng offset(S2LatLng const& ll, float d_north_m, float d_east_m){
    float scaling = 1/0.011131884502145034;
    float d_lat = d_north_m*scaling;
    float d_lng = d_east_m*scaling/longitude_scale(ll);
    return S2LatLng::FromE7(
        ll.lat().e7()+d_lat,
        ll.lng().e7()+d_lng
    );
}

float to_deg(float radians){
    return (radians*180)/M_PI;
}

float to_rad(float degrees){
    return (degrees*M_PI)/180;
}

S2LatLng offset_bearing(S2LatLng const& ll, float bearing, float dist_m){
    float d_north = cosf(to_rad(bearing))*dist_m;
    float d_east = sinf(to_rad(bearing))*dist_m;
    return offset(ll, d_north, d_east);
}

// bearing angle between two latlngs
float bearing_to(S2LatLng const& ll1, S2LatLng const& ll2){
    float off_x = ll2.lng().degrees() - ll1.lng().degrees();
    float off_y = (ll2.lat().degrees() - ll1.lat().degrees())/longitude_scale(ll2);
    float bearing = 90 + to_deg(atan2f(-off_y, off_x));
    if(bearing < 0) bearing += 360;
    return bearing;
}

float clamp(float value, float min, float max){
    if(value < min){
        return min;
    } else if(value > max){
        return max;
    }
    return value;
}



#endif