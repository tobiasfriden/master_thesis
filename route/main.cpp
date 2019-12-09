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
#include <chrono>

#include "astar.h"
#include "simulation.h"
#include "motion_primitive.h"
#include "hlut.h"
#include "obstacle.h"
#include "landing.h"

using namespace mapbox::geojson;

double best_approach_heading(Landing& land, const Obstacles& obst, const HLUT& hlut, double start_hdg){
    double safety_h = Constants::safety_alt();
    double alt = Constants::start_alt();

    Coordinate start(0, 0, start_hdg);

    double best_cost = std::numeric_limits<double>().max();
    double best_hdg=-1, hdg, hlut_cost, cost;
    Vector2_d ap;
    for(int i=0; i<35; i++){
        hdg = i*10;
        if(land.feasible_heading(obst, hdg, safety_h, alt)){
            std::cout << "feasible: " << hdg << "land dist: " << land.land_distance(hdg, alt) << std::endl;
            cost = land.land_distance(hdg, alt);
            if(cost < best_cost){
                best_cost = cost;
                best_hdg = hdg;
            }
        }
    }
    return best_hdg;
}

int main(int argc, char**argv){

    S2LatLng origin = offset(S2LatLng::FromDegrees(57.6432, 11.8630), Constants::start_offset(), 0);

    Simulator sim;

    MotionPrimitiveSet primitives(5);
    primitives.load_from_file("../primitives/");

    HLUT hlut(5);
    hlut.load_binary("../hlut/");

    Obstacles obst(origin, Constants::safety_dist(), "../geodata/single.geojson");
    obst.save_to_file(origin, "../obstacles.txt");

    Landing land(Vector2_d(-450, 0), 100, 200, 30);
    land.save_to_file("../landing.txt");

    double best_hdg = best_approach_heading(land, obst, hlut, 0);
    if(best_hdg < 0){
        std::cerr << "no feasible directions found! " << std::endl;
        return -1;
    }
    std::cout << "best heading: " << best_hdg << std::endl;
    Landing::line approach = land.optimize(best_hdg);
    std::cout << "approach point: " << approach[0] << std::endl;
    
    Vector2_d goal = approach[0];
    if(obst.in_collision(goal)){
        std::cerr << "goal in collision" << std::endl;
    }
    auto path = astar(sim, primitives, hlut, obst, origin, goal, best_hdg, 1);
    //if(path.size() > 2) path = filter_solution(obst, sim, path);
    path.pop_back();
    path.push_back(Coordinate(approach[0].x(), approach[0].y(), 0));
    path.push_back(Coordinate(approach[1].x(), approach[1].y(), 0));

    std::ofstream out;
    out.open("../sol.txt");

    feature_collection points;
    for(auto p : path) {
        feature f;
        auto wp = p.waypoint();
        auto ll = offset(origin, wp.x(), wp.y());
        f.geometry = point(ll.lng().degrees(), ll.lat().degrees());
        points.push_back(f);
    }
    out << stringify(points) << std::endl;
    out.close();

    std::vector<Vector2_d> mission;
    std::transform(
        path.begin(),
        path.end(),
        std::back_inserter(mission),
        [](Coordinate c) -> Vector2_d {
            return c.waypoint();
        }
    );
    std::cout << "sim mission: " << mission.size() << std::endl;
    auto trajectory = sim.simulate_mission(mission);
    std::cout << "done" << std::endl;
    sim.save_trajectory(origin, trajectory, "../sim.txt");

}

// geojson readGeoJSON(const std::string &path, bool use_convert) {
//     std::ifstream t(path.c_str());
//     std::stringstream buffer;
//     buffer << t.rdbuf();
//     return parse(buffer.str());
// }
 
// void buildIndex(geojson geo, MutableS2ShapeIndex& index) {
//     const auto& features = geo.get<feature_collection>();
//     std::vector<S2Point> pVector;
//     for(const auto& f: features) {
//         pVector.clear();
//         const auto& p = f.geometry.get<polygon>();
//         const auto& ring = p.at(0);
//         for (auto coord: ring) {
//             pVector.push_back(S2LatLng::FromDegrees(coord.y, coord.x).ToPoint().Normalize()); 
//         }
//         pVector.pop_back();
//         index.Add(absl::make_unique<S2Loop::Shape>(new S2Loop(pVector)));
//     }
// }

// int main(int argc, char* argv[]) {
//     const auto& geo = readGeoJSON(argv[1], false);
//     MutableS2ShapeIndex index;

//     buildIndex(geo, index);
    
//     std::cout << "Create crossing edge q" << std::endl;
//     auto query = new S2CrossingEdgeQuery(&index);
//     std::cout << "Done!" << std::endl;

//     std::cout << "Create closest edge q" << std::endl;
//     auto dquery = new S2ClosestEdgeQuery(&index);
//     std::cout << "Done!" << std::endl;

//     Grid grid(query, dquery, 25);
//     std::cout << "Calculate path!" << std::endl;
//     auto path = astar(
//         &grid,
//         S2LatLng::FromDegrees(57.60311775383921, 11.820602416992188),
//         S2LatLng::FromDegrees(57.571281208449136, 11.714344024658203)
//     );
//     std::cout << path.size() << std::endl;

//     feature_collection points;
//     for(auto p : path) {
//         feature f;
//         auto ll = p.latLng();
//         f.geometry = point(ll.lng().degrees(), ll.lat().degrees());
//         points.push_back(f);
//     }
//     std::cout << stringify(points) << std::endl;

// }


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