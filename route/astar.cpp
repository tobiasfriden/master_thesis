#include <astar.h>

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
    const Obstacles& obst,
    const S2LatLng& origin,
    const Vector2_d& to,
    double goal_hdg,
    double inflation
){
    auto start_time = std::chrono::system_clock::now();
    ClosedSet closed_set;
    Frontier frontier;
    std::vector<Coordinate> nbrVec;
    std::vector<Coordinate> pathVec;
    Vector2_d from(0, 0);

    Coordinate start(from, from, 0, 0, 0);
    frontier.push(start, nullptr, 0);

    Coordinate goal(to, to, goal_hdg, goal_hdg, 0);
    double h;
    hlut.lookup_cost(start, goal, h);
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

        bool found_in_expansion = false;
        Node::Ptr goal_node;
        if(get_distance_NE(current -> coord.position(), goal.position()).Norm() < 300 && !obst.in_collision(current -> coord.waypoint(), goal.waypoint())){
            //Expansion towards final goal
            sim.reset(
                current ->coord.position().x(),
                current ->coord.position().y(),
                current ->coord.heading()
            );
            std::vector<Vector2_d> traj;
            double cost = sim.simulate_waypoints(current -> coord.waypoint(), goal.waypoint(), traj, false);
            Coordinate coord(goal.waypoint(), sim.pos(), sim.yaw(), sim.path_bearing(), current -> coord.cost() + cost);
            coord.add_states(traj);
            if(coord == goal){
                goal_node = std::make_shared<Node>(coord);
                std::cout << "found in goal expansion" << std::endl;
                goal_node -> parent = current;
                found_in_expansion = true;
            }
        }

        if(current -> coord == goal || found_in_expansion) {
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
            if(found_in_expansion){
                np = goal_node;
            }
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
        auto coords = current -> coord.get_neighbours(sim, primitives, obst);
        //auto coords = current -> coord.get_mp_neighbours(primitives, obst, sim.wind_dir());
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
 
