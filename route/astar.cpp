#include <vector>
#include <iostream>
#include <memory>

#include <s2/s2closest_edge_query.h>
#include <s2/s2latlng.h>
#include <boost/heap/priority_queue.hpp>

#include <astar.h>

// Grid definitions
bool Grid::isCrossing(const S2LatLng& ll1, const S2LatLng& ll2) const {
    auto target = S2ClosestEdgeQuery::PointTarget(ll2.ToPoint());
    auto res = _query -> GetCrossingEdges(
        ll1.ToPoint().Normalize(),
        ll2.ToPoint().Normalize(),
        S2CrossingEdgeQuery::CrossingType::ALL
    );
    return res.size() > 0;
}

std::vector<S2LatLng> Grid::getNeighbours(const S2LatLng& ll) const {
    std::vector<S2LatLng> coords;
    auto target = S2ClosestEdgeQuery::PointTarget(ll.ToPoint().Normalize());
    int dynamic_step = _dQuery -> GetDistance(&target).e6()/20;
    std::cout << "Dyn_step: " << dynamic_step << std::endl;
    dynamic_step = std::max(dynamic_step, step);
    for(int i=0; i<8; i++) {
        int32 inc_lat = _inc[i][0]*dynamic_step;
        int32 inc_lng = _inc[i][1]*dynamic_step;
        S2LatLng ll_new =
            S2LatLng::FromE6(
                ll.lat().e6() + inc_lat,
                ll.lng().e6() + inc_lng
            );
        if(!isCrossing(ll, ll_new)){
            coords.push_back(ll_new);
        }
    }

    return coords;
}

// Coordinate definitions
std::vector<Coordinate> Coordinate::getNeighbours(
    Simulator& sim,
    MotionPrimitiveSet& primitives
) {
    std::vector<Coordinate> coordVector; 
    std::vector<S2LatLng> traj;
    coordVector.clear();
    auto offsets = primitives.get_expansions(_heading, sim.wind_dir());
    S2LatLng start = _latLng;
    S2LatLng start_waypoint = _waypoint;
    float primitive_cost;

    for(auto ofs: offsets){
        traj.clear();
        S2LatLng goal = offset(start, ofs[0], ofs[1]);
        sim.reset(start.lat().degrees(), start.lng().degrees(), _heading);
        primitive_cost = sim.simulate_waypoints(start_waypoint, goal, traj, false);
        Coordinate coord(goal, sim.pos(), sim.yaw(), _cost + primitive_cost);
        coord.add_states(traj);
        coordVector.push_back(coord);
    }
    return coordVector;
}

void Coordinate::add_states(std::vector<S2LatLng> new_states){
    //_states.clear();
    //_states.reserve(new_states.size());
    std::copy(new_states.begin(), new_states.end(), std::back_inserter(_states));
}

mapbox::geojson::feature_collection Coordinate::get_states(){
    mapbox::geojson::feature_collection fc;
    for(auto s : _states){
        mapbox::geojson::feature f;
        f.geometry = mapbox::geojson::point(s.lng().degrees(), s.lat().degrees());
        fc.push_back(f);
    }
    return fc;
}

float Coordinate::heuristic(const Coordinate& to) {
    float bearing = bearing_to(_latLng, to.latLng());
    float bearing_diff = fabs(bearing - _heading);
    bearing_diff = fmin(bearing_diff, fabs(360-bearing_diff));
    return get_distance_NE(_latLng, to.latLng()).Norm() + bearing_diff;
}

bool Coordinate::isEqual(const Coordinate& to) const {
    const S2LatLng& ll = to.latLng();
    float yaw_diff = fabs(_heading - to.heading());
    return _latLng.lat().e5()/50 == ll.lat().e5()/50 &&
           _latLng.lng().e5()/50 == ll.lng().e5()/50 &&
          (yaw_diff < 10 || fabs(360-yaw_diff) < 10);
}

// Node definition
bool Node::operator==(const Node& other) const {
    return coord == other.coord;
}

bool operator<(const Node& n1, const Node& n2) {
    return n1.rank < n2.rank;
}

// NodeQueue definition
const Node::Ptr NodeQueue::top() {
    auto np = _nq.top();
    _nq.pop();
    _hm.erase(np -> coord);
    return np;
}

void NodeQueue::push(const Node::Ptr& np) {
    auto handle = _nq.push(np);
    _hm[np -> coord] = handle;
}

void NodeQueue::update(const Node::Ptr& np) {
    std::cout << "update" << std::endl;
    std::cout << (np == nullptr) << std::endl;  
    if(_hm.find(np->coord) != _hm.end()) {
        std::cout << "found" << std::endl;
        std::cout << np->coord.latLng() << std::endl;
        std::cout << "test" << std::endl;
        auto test = _hm[np->coord];
        std::cout << "did not crash" << std::endl;
        _nq.erase(_hm[np->coord]);
        std::cout << "got here" << std::endl;
        push(np);
    } else {
        push(np);
    }
    std::cout << "update done" << std::endl;
}

bool NodeQueue::empty() {
    return _nq.empty();
}

// NodeMap definition
const Node::Ptr NodeMap::get(const Coordinate& c) {
    Node::Ptr n;
    if(_nmap.find(c) == _nmap.end()) {
        n = std::make_shared<Node>(c);
        _nmap[c] = n;
    } else {
        n = _nmap[c];
    }
    return n;
}

void NodeMap::save(std::string path) {
    std::ofstream out;
    out.open(path.c_str());
    for(auto it = _nmap.begin(); it != _nmap.end(); it++){
        auto record = *it;
        auto coord = record.first.latLng();
        out << coord.lat().degrees() << " " << coord.lng().degrees() << std::endl;
    }
    out.close();
}

// A^* implementation
std::vector<Coordinate> astar(
    Simulator& sim,
    MotionPrimitiveSet& primitives,
    const S2LatLng& from,
    const S2LatLng& to,
    float goal_hdg
){
    NodeMap nm;
    NodeQueue pq;
    std::vector<Coordinate> nbrVec;
    std::vector<Coordinate> pathVec;

    Node::Ptr fromNode = nm.get(Coordinate(from, from, 0, 0));
    fromNode -> open = true;
    pq.push(fromNode);

    Node::Ptr toNode = nm.get(Coordinate(to, to, goal_hdg, 0));
    float best_heuristic = std::numeric_limits<float>().max();
    Node::Ptr best_node;

    while(true) {
        if(pq.empty()) {
            std::cout << "No path found" << std::endl;
            return pathVec;
        }

        Node::Ptr current = pq.top();
        current -> open = false;
        current -> closed = true;

        // Expansion towards final goal
        // sim.reset(
        //     current ->coord.latLng().lat().degrees(),
        //     current ->coord.latLng().lng().degrees(),
        //     current ->coord.heading()
        // );
        // sim.simulate_waypoints(current -> coord.latLng(), to, false);
        // float bearing_diff = fabs(sim.yaw() - goal_hdg);
        // float xtrack = fabs(sim.xtrack_error());
        // bool found_in_expansion = false;
        // Node::Ptr goal_node;
        // if(xtrack < 5 && (bearing_diff < 15 || fabs(360-bearing_diff) < 15)){
        //     std::cout << "found in goal expansion" << std::endl;
        //     goal_node = nm.get(Coordinate(to, sim.pos(), sim.yaw(), 0));
        //     goal_node -> parent = current;
        //     found_in_expansion = true;
        // }
        bool found_in_expansion = false;
        Node::Ptr goal_node;

        if(found_in_expansion || current -> coord.isEqual(toNode -> coord) || best_heuristic <= 10) {
            std::cout << "Path found" << std::endl;
            std::cout << get_distance_NE(from, current -> coord.latLng()) << std::endl;
            Node::Ptr np;
            if(found_in_expansion){
                np = goal_node;
            } else if(best_heuristic <= 10){
                np = best_node;
            } else {
                np = current;
            }
            mapbox::geojson::feature_collection fc;
            auto f = np -> coord.get_states();
            std::copy(f.begin(), f.end(), std::back_inserter(fc));
            while(np != nullptr) {
                pathVec.push_back(np -> coord);
                f = np->coord.get_states();
                std::copy(f.begin(), f.end(), std::back_inserter(fc));
                np = np -> parent;
            }
            std::ofstream out;
            out.open("../sim_astar.txt");
            out << mapbox::geojson::stringify(fc);
            out.close();
            nm.save("../visited.txt");
            std::reverse(pathVec.begin(), pathVec.end());
            return pathVec;
        }
        auto coords = current -> coord.getNeighbours(sim, primitives);
        for(auto c: coords) {
            auto ofs = get_distance_NE(S2LatLng::FromDegrees(0, 0), c.latLng());
            // if(fabs(c.heading())<5 && fabs(ofs[1])<5){
            //     std::cout << "coord: " << ofs << std::endl;
            //     std::cout << "heading: " << c.heading() << std::endl;
            //     std::cout << "cost: " << c.cost() << std::endl;
            // }

            auto nbrNode = nm.get(c);
            if(c.cost() < nbrNode -> cost) {
                nbrNode -> open = false;
                nbrNode -> closed = false;
            }
            if(!(nbrNode -> open) && !(nbrNode -> closed)) {
                nbrNode -> cost = c.cost();
                nbrNode -> open = true;
                nbrNode -> rank = c.cost() + 5*c.heuristic(toNode -> coord);
                float hdg_diff = fabs(c.heading() - goal_hdg);
                if((hdg_diff < 10 || fabs(360-hdg_diff) < 10) && c.heuristic(toNode -> coord) < best_heuristic){
                    best_heuristic = c.heuristic(toNode -> coord);
                    best_node = nbrNode;
                    std::cout << "best h: " << best_heuristic << std::endl;
                }
                nbrNode -> parent = current;
                nbrNode -> coord = c;
                pq.update(nbrNode);
            }
        }
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
    float prev_bearing = bearing_to(start.latLng(), next.latLng());
    for(; it != pathVec.end(); it++){
        Coordinate last = *it;
        float bearing_first = fabs(bearing_to(start.latLng(), next.latLng())-prev_bearing);
        bearing_first = fmin(bearing_first, fabs(360-bearing_first));
        float bearing_second = fabs(bearing_to(next.latLng(), last.latLng())-prev_bearing);
        bearing_second = fmin(bearing_second, fabs(360-bearing_second));
        std::cout << "first: " << bearing_first << std::endl;
        std::cout << "second: " << bearing_second << std::endl;
        std::cout << "dist: " << get_distance_NE(start.latLng(), last.latLng()).Norm() << std::endl;
        float dist = get_distance_NE(start.latLng(), last.latLng()).Norm();
        if(dist > 20 && (bearing_first > 20 || bearing_second > 20)){
            filtered.push_back(next);
            prev_bearing = fabs(bearing_to(start.latLng(), last.latLng()));
            std::cout << " prev_bearing: " << prev_bearing << std::endl;
        }
        start = next;
        next = last;
    }
    filtered.push_back(next);
    return filtered;
}
 
