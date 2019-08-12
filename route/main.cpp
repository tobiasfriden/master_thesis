#include <mapbox/geojson.hpp>
#include <mapbox/geojson_impl.hpp>
#include <mapbox/geometry.hpp>
#include <s2/s2latlng.h>
#include <s2/s2point.h>
#include <s2/s2loop.h>
#include <s2/s2polygon.h>
#include <s2/s2polyline.h>
#include <s2/s2closest_edge_query.h>
#include <s2/s2crossing_edge_query.h>
#include <s2/s2contains_point_query.h>
#include <s2/s2shape_index.h>
#include <s2/s2shape.h>
#include <s2/third_party/absl/memory/memory.h>
#include <s2/s2pointutil.h>

#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>

#include <astar.h>

using namespace mapbox::geojson;

geojson readGeoJSON(const std::string &path, bool use_convert) {
    std::ifstream t(path.c_str());
    std::stringstream buffer;
    buffer << t.rdbuf();
    return parse(buffer.str());
}
 
void buildIndex(geojson geo, MutableS2ShapeIndex& index) {
    const auto& features = geo.get<feature_collection>();
    std::vector<S2Point> pVector;
    for(const auto& f: features) {
        pVector.clear();
        const auto& p = f.geometry.get<polygon>();
        const auto& ring = p.at(0);
        for (auto coord: ring) {
            pVector.push_back(S2LatLng::FromDegrees(coord.y, coord.x).ToPoint().Normalize()); 
        }
        pVector.pop_back();
        index.Add(absl::make_unique<S2Loop::Shape>(new S2Loop(pVector)));
    }
}

int main(int argc, char* argv[]) {
    const auto& geo = readGeoJSON(argv[1], false);
    MutableS2ShapeIndex index;

    buildIndex(geo, index);
    
    std::cout << "Create crossing edge q" << std::endl;
    auto query = new S2CrossingEdgeQuery(&index);
    std::cout << "Done!" << std::endl;

    std::cout << "Create closest edge q" << std::endl;
    auto dquery = new S2ClosestEdgeQuery(&index);
    std::cout << "Done!" << std::endl;

    Grid grid(query, dquery, 25);
    std::cout << "Calculate path!" << std::endl;
    auto path = astar(
        &grid,
        S2LatLng::FromDegrees(57.60311775383921, 11.820602416992188),
        S2LatLng::FromDegrees(57.571281208449136, 11.714344024658203)
    );
    std::cout << path.size() << std::endl;

    feature_collection points;
    for(auto p : path) {
        feature f;
        auto ll = p.latLng();
        f.geometry = point(ll.lng().degrees(), ll.lat().degrees());
        points.push_back(f);
    }
    std::cout << stringify(points) << std::endl;

}


/* 
int main() {
    const auto& geo = readGeoJSON("test.geojson", false);
    const auto& features = geo.get<feature_collection>();

    std::vector<S2Point> pVector;
    for (auto f : features) {
        const auto& p = f.geometry.get<polygon>();
        const auto& ring = p.at(0);
        for (auto coord: ring) {
            S2Point p = S2LatLng::FromDegrees(coord.y, coord.x).ToPoint();
            pVector.push_back(p);
        }
        pVector.pop_back();
    }
    S2Loop loop(pVector);
    S2Polygon poly(std::make_unique<S2Loop>(pVector));

    MutableS2ShapeIndex index;
    index.Add(std::make_unique<S2Polygon::Shape>(&poly));

    S2ClosestEdgeQuery::EdgeTarget target(
        S2LatLng::FromDegrees(
            57.64484946788994,
            11.689109802246092
        ).ToPoint(),
        S2LatLng::FromDegrees(
            57.65770860918321,
            11.731681823730469
        ).ToPoint()
    );
    S2ClosestEdgeQuery query(&index);
    auto results = query.FindClosestEdges(&target);

    for (auto res : results) {
        std::cout << res.distance() << std::endl;
    }

    
    auto closest = query.GetEdge(res);

    feature_collection points;

    feature p1;
    S2LatLng p1_ll(closest.v0);
    p1.geometry = point(p1_ll.lng().degrees(), p1_ll.lat().degrees());
    points.push_back(p1);

    feature p2;
    S2LatLng p2_ll(closest.v1);
    p2.geometry = point(p2_ll.lng().degrees(), p2_ll.lat().degrees());
    points.push_back(p2);

    std::cout << stringify(points) << std::endl;
    
    return 0;
}*/