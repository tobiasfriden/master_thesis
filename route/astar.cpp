#include <astar.h>

// // Grid definitions
// bool Grid::isCrossing(const S2LatLng& ll1, const S2LatLng& ll2) const {
//     auto target = S2ClosestEdgeQuery::PointTarget(ll2.ToPoint());
//     auto res = _query -> GetCrossingEdges(
//         ll1.ToPoint().Normalize(),
//         ll2.ToPoint().Normalize(),
//         S2CrossingEdgeQuery::CrossingType::ALL
//     );
//     return res.size() > 0;
// }

// std::vector<S2LatLng> Grid::getNeighbours(const S2LatLng& ll) const {
//     std::vector<S2LatLng> coords;
//     auto target = S2ClosestEdgeQuery::PointTarget(ll.ToPoint().Normalize());
//     int dynamic_step = _dQuery -> GetDistance(&target).e6()/20;
//     std::cout << "Dyn_step: " << dynamic_step << std::endl;
//     dynamic_step = std::max(dynamic_step, step);
//     for(int i=0; i<8; i++) {
//         int32 inc_lat = _inc[i][0]*dynamic_step;
//         int32 inc_lng = _inc[i][1]*dynamic_step;
//         S2LatLng ll_new =
//             S2LatLng::FromE6(
//                 ll.lat().e6() + inc_lat,
//                 ll.lng().e6() + inc_lng
//             );
//         if(!isCrossing(ll, ll_new)){
//             coords.push_back(ll_new);
//         }
//     }

//     return coords;
// }

bool is_equal_goal(Coordinate const& c1, Coordinate const& goal){
    Vector2_d diff = c1.position() - goal.position();
    double heading_diff = std::abs(c1.bearing() - goal.bearing());
    heading_diff = std::min(heading_diff, std::abs(360-heading_diff));
    return std::abs(diff.x()) < Constants::goal_size &&
           std::abs(diff.y()) < Constants::goal_size &&
           heading_diff < Constants::hdg_size;
}

// A^* implementation
std::vector<Coordinate> astar(
    Simulator& sim,
    const MotionPrimitiveSet& primitives,
    const HLUT& hlut,
    const S2LatLng& origin,
    const Vector2_d& to,
    double goal_hdg,
    double inflation
){
    auto start_time = std::chrono::system_clock::now();
    ob::StateSpacePtr space(new ob::DubinsStateSpace(1.5*sim.airspeed()/sim.yrate_max()));
    ClosedSet closed_set;
    Frontier frontier;
    std::vector<Coordinate> nbrVec;
    std::vector<Coordinate> pathVec;
    Vector2_d from(0, 0);

    Coordinate start(from, from, 0, 0, 0);
    frontier.push(start, nullptr, 0);

    Coordinate goal(to, to, goal_hdg, goal_hdg, 0);
    double best_heuristic = std::numeric_limits<double>().max();
    double h;
    hlut.lookup_cost(start, goal, h);
    //double h = wind_corrected_distance(start.position(), goal.position());
    std::cout << "heuristic estimate: " << h << std::endl;
    int calls = 0;

    while(true) {
        
        if(frontier.empty()) {
            std::cout << "No path found" << std::endl;
            return pathVec;
        }

        Node::Ptr current = frontier.top();
        if(closed_set.is_visited(current->coord)){
            std::cout << "already visited: " << current -> coord;
            continue;
        }

        if(is_equal_goal(current -> coord, goal)) {
            auto end_time = std::chrono::system_clock::now();
            std::chrono::duration<double> diff = end_time - start_time;
            std::cout << "Path found in: " << diff.count() << std::endl;
            std::cout << "heuristic calls: " << calls << std::endl;
            std::cout << "expanded nodes: " << closed_set.size() << std::endl;
            std::cout << "final heading: " << current -> coord.heading() << std::endl;
            std::cout << "final bearing: " << current -> coord.bearing() << std::endl;
            std::cout << "total cost: " << current -> coord.cost() << std::endl;
            std::cout << get_distance_NE(from, current -> coord.position()) << std::endl;
            Node::Ptr np = current;
            mapbox::geojson::feature_collection fc;
            auto f = np -> coord.get_states(origin);
            std::copy(f.begin(), f.end(), std::back_inserter(fc));
            while(np != nullptr) {
                pathVec.push_back(np -> coord);
                f = np->coord.get_states(origin);
                std::copy(f.begin(), f.end(), std::back_inserter(fc));
                np = np -> parent;
            }
            std::ofstream out;
            out.open("../sim_astar.txt");
            out << mapbox::geojson::stringify(fc);
            out.close();
            closed_set.save("../visited.txt");
            std::reverse(pathVec.begin(), pathVec.end());
            return pathVec;
        }
        auto coords = current -> coord.get_neighbours(sim, primitives);
        //auto coords = current -> coord.get_mp_neighbours(primitives, sim.wind_dir());
        for(auto c: coords) {
            if(c == current->coord){
                continue;
            }
            if(!closed_set.is_visited(c)) {
                double cost;
                hlut.lookup_cost(c, goal, cost);
                //double cost = wind_corrected_distance(c.position(), goal.position());
                // double cost = (c.position() - goal.position()).Norm();
                // double rank = inflation*c.heuristic(goal, space, calls)
                frontier.push(c, current, c.cost() + inflation*cost);
            }
        }
        closed_set.set_visited(current -> coord);
    }
}

std::vector<Coordinate> filter_solution(std::vector<Coordinate> pathVec){
    std::vector<Coordinate> filtered;

    auto it = pathVec.begin();
    Coordinate start = *it;
    filtered.push_back(start);
    it++;
    Coordinate next = *it;
    it++;
    double prev_bearing = bearing_to(start.position(), next.position());
    for(; it != pathVec.end(); it++){
        Coordinate last = *it;
        double bearing_first = fabs(bearing_to(start.position(), next.position())-prev_bearing);
        bearing_first = fmin(bearing_first, fabs(360-bearing_first));
        double bearing_second = fabs(bearing_to(next.position(), last.position())-prev_bearing);
        bearing_second = fmin(bearing_second, fabs(360-bearing_second));
        double dist = get_distance_NE(start.position(), last.position()).Norm();
        if(dist > 20 && (bearing_first > 20 || bearing_second > 20)){
            filtered.push_back(next);
            prev_bearing = fabs(bearing_to(start.position(), last.position()));
        }
        start = next;
        next = last;
    }
    filtered.push_back(next);
    return filtered;
}
 
