#include <mapbox/geojson.hpp>
#include <mapbox/geojson_impl.hpp>
#include <mapbox/geometry.hpp>

#include "simulation.h"

using namespace mapbox::geojson;

int main(int argc, char** argv){
    std::stringstream ss;
    ss << "../" << argv[1] << ".txt";
    std::string infile = ss.str();

    std::ifstream in(infile.c_str());
    std::stringstream buffer;
    buffer << in.rdbuf();
    geojson fc = parse(buffer.str());

    const auto& features = fc.get<feature_collection>();

    auto current = features.front().geometry.get<point>();
    
    S2LatLng origin = S2LatLng::FromDegrees(current.y, current.x);
    std::vector<Vector2_d> mission;
    for(auto f : features){
        auto p = f.geometry.get<point>();
        mission.push_back(get_distance_NE(
            origin,
            S2LatLng::FromDegrees(p.y, p.x)
        ));
    }

    Simulator sim;
    auto traj = sim.simulate_mission(mission);
    std::stringstream out_ss;
    out_ss << "../" << argv[1] << "_traj.txt";
    sim.save_trajectory(origin, traj, out_ss.str().c_str());
}