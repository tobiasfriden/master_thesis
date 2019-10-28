#ifndef OBSTACLE_H_
#define OBSTACLE_H_

#include <fstream>

#include <mapbox/geojson.hpp>
#include <mapbox/geojson_impl.hpp>
#include <mapbox/geometry.hpp>

#include <s2/s2latlng.h>
#include <s2/s2point.h>
#include <s2/s2loop.h>
#include <s2/s2polygon.h>
#include <s2/s2polyline.h>
#include <s2/s2closest_edge_query.h>
#include <s2/s2contains_point_query.h>
#include <s2/s2shape_index.h>
#include <s2/s2shape.h>
#include <s2/s2earth.h>
#include <s2/third_party/absl/memory/memory.h>
#include <s2/s2pointutil.h>

#include "geo_utils.h"

using namespace mapbox::geojson;

class Obstacles{
public:
    Obstacles(const S2LatLng& origin, double safety_m, std::string path);

    void save_to_file(const S2LatLng& origin, std::string path);

    bool in_collision(const Vector2_d& coord) const;
    bool in_collision(const Vector2_d& start, const Vector2_d& goal) const;

private:
    void build_index(std::string path);
    geojson read_features(std::string path);
    S2LatLng to_latlng(const Vector2_d& pos) const;

    S2LatLng _origin;
    S1ChordAngle _safety_dist;
    MutableS2ShapeIndex index;
    S2ClosestEdgeQuery* edge_q;
    S2ContainsPointQuery<MutableS2ShapeIndex>* point_q;
};


#endif