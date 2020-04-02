#include <iostream>
#include <s2/s2latlng.h>
#include <mapbox/geojson.hpp>
#include <mapbox/geojson_impl.hpp>
#include <mapbox/geometry.hpp>

#include "simulation.h"

using namespace mapbox::geojson;

void simulate_mission(){
    S2LatLng origin = S2LatLng::FromDegrees(57.485645,11.930835);

    std::vector<Vector2_d> mission;

    std::ifstream in("sol.txt");
    std::stringstream buffer;
    buffer << in.rdbuf();

    geojson geo = parse(buffer.str());

    const auto& features = geo.get<feature_collection>();

    for(const auto& f: features){
        const auto& p = f.geometry.get<point>();
        S2LatLng point = S2LatLng::FromDegrees(p.y, p.x);
        mission.push_back(get_distance_NE(origin, point));
    }
    double init_heading = bearing_to(mission[0], mission[1]);
    Vector2_d start = mission[1];
    Simulator sim(14, 8.24, 117.785, 0.6);
    sim.reset(start.x(), start.y(), init_heading);
    std::vector<Vector2_d> sim_mission(mission.begin()+1, mission.end());
    std::vector<Vector2_d> traj = sim.simulate_mission(sim_mission);

    sim.save_trajectory(origin, traj, "simulated_mission.txt");
}

int main(){
    simulate_mission();
}