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
#include "waypoint_opt.h"

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

void print_sol(S2LatLng origin, std::vector<Coordinate> path, std::string file){

    std::ofstream out;
    out.open(file);

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
}

std::vector<Vector2_d> coords_to_vec(std::vector<Coordinate> coords){
    std::vector<Vector2_d> mission;
    std::transform(
        coords.begin(),
        coords.end(),
        std::back_inserter(mission),
        [](Coordinate c) -> Vector2_d {
            return c.waypoint();
        }
    );
    return mission;
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
    
    print_sol(origin, path, "../sol_unfiltered.txt");
    sim.reset(0, 0, 0, 0);
    auto trajectory = sim.simulate_mission(coords_to_vec(path));
    sim.save_trajectory(origin, trajectory, "../sim_unfiltered.txt");

    // if(path.size() > 2) path = filter_solution(obst, sim, path);
    // print_sol(origin, path, "../sol_filtered.txt");
    // sim.reset(0, 0, 0, 0);
    // trajectory = sim.simulate_mission(coords_to_vec(path));
    // sim.save_trajectory(origin, trajectory, "../sim_filtered.txt");

    WaypointOpt wp_opt(0, best_hdg + 180);
    if(path.size() > 2) path = wp_opt.improve_path(sim, obst, path);
    
    path.pop_back();
    path.push_back(Coordinate(approach[0].x(), approach[0].y(), 0));
    path.push_back(Coordinate(approach[1].x(), approach[1].y(), 0));

    print_sol(origin, path, "../sol.txt");
    sim.reset(0, 0, 0, 0);
    trajectory = sim.simulate_mission(coords_to_vec(path));
    sim.save_trajectory(origin, trajectory, "../sim.txt");
    sim.save_rolls("../sim_att.csv");
}
