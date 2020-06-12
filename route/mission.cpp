#include <iostream>
#include <s2/s2latlng.h>
#include <mapbox/geojson.hpp>
#include <mapbox/geojson_impl.hpp>
#include <mapbox/geometry.hpp>

#include "simulation.h"

using namespace mapbox::geojson;

void simulate_mission(){
    S2LatLng origin = offset(S2LatLng::FromDegrees(57.6432, 11.8630), Constants::start_offset(), 0);

    std::vector<Vector2_d> mission;

    std::ifstream in("plot_data/roll/sol.txt");
    std::stringstream buffer;
    buffer << in.rdbuf();

    geojson geo = parse(buffer.str());
    const auto& features = geo.get<feature_collection>();

    for(const auto& f: features){
        const auto& p = f.geometry.get<point>();
        S2LatLng point = S2LatLng::FromDegrees(p.y, p.x);
        mission.push_back(get_distance_NE(origin, point));
    }
    Simulator sim;
    double init_heading = wind_correction_angle(0, Constants::airspeed(), Constants::wind_dir(), Constants::wind_spd());
    std::cout << "wca: " << init_heading << std::endl;
    sim.reset(0, 0, init_heading, 0);
    std::vector<Vector2_d> traj = sim.simulate_mission(mission);

    sim.save_trajectory(origin, traj, "plot_data/roll/sim.txt");
}

int main(){
    simulate_mission();
}