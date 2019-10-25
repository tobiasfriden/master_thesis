#include "graph.h"

// Coordinate definitions

std::vector<Coordinate> Coordinate::get_neighbours(
    Simulator& sim,
    const MotionPrimitiveSet& primitives
) {
    std::vector<Coordinate> coordVector; 
    std::vector<Vector2_d> traj;

    auto offsets = primitives.get_expansions(_heading, sim.wind_dir());
    Vector2_d start = _position;
    Vector2_d start_waypoint = _waypoint;
    double primitive_cost;

    for(auto ofs: offsets){
        traj.clear();
        Vector2_d goal_waypoint = start_waypoint + Vector2_d(ofs[0], ofs[1]);
        sim.reset(start.x(), start.y(), _heading);
        primitive_cost = sim.simulate_waypoints(start_waypoint, goal_waypoint, traj, false);
        Coordinate coord(goal_waypoint, sim.pos(), sim.yaw(), sim.path_bearing(), _cost + primitive_cost);
        coord.add_states(traj);
        coordVector.push_back(coord);
    }
    return coordVector;
}

std::vector<Coordinate> Coordinate::get_mp_neighbours(const MotionPrimitiveSet& primitives, int wind_dir){
    std::vector<Coordinate> coord_vec;

    auto mps = primitives.get_mp_expansions(_heading, wind_dir);
    Vector2_d start = _position;
    Vector2_d start_waypoint = _waypoint;
    for(auto mp: mps){
        Vector2_d goal_waypoint = start_waypoint + Vector2_d(mp.wp_north(), mp.wp_east());
        Vector2_d goal = start + Vector2_d(mp.north(), mp.east());
        Coordinate coord(
            goal_waypoint,
            goal,
            mp.heading(),
            0,
             _cost + mp.cost()
        );
        coord_vec.push_back(coord);
    }
    return coord_vec;
}

void Coordinate::add_states(std::vector<Vector2_d> new_states){
    _states.clear();
    std::copy(new_states.begin(), new_states.end(), std::back_inserter(_states));
}

mapbox::geojson::feature_collection Coordinate::get_states(S2LatLng const& origin){
    mapbox::geojson::feature_collection fc;
    for(auto state : _states){
        S2LatLng s = offset(origin, state.x(), state.y());
        mapbox::geojson::feature f;
        f.geometry = mapbox::geojson::point(s.lng().degrees(), s.lat().degrees());
        fc.push_back(f);
    }
    return fc;
}

double Coordinate::heuristic(const Coordinate& to, const ob::StateSpacePtr& space, int& calls) {
    // if(is_equal_goal(*this, to)){
    //     return 0;
    // }
    //return (to.position() - _position).Norm();
    calls++;
    ob::ScopedState<ob::DubinsStateSpace> start(space);
    ob::ScopedState<ob::DubinsStateSpace> goal(space);

    start->setX(_position.x());
    start->setY(_position.y());
    start->setYaw(to_rad(_heading));

    goal->setX(to.position().x());
    goal->setY(to.position().y());
    goal->setYaw(to_rad(to.heading()));
    return space -> distance(start(), goal());
}

// Frontier definition
const Node::Ptr Frontier::top() {
    auto np = _nq.top();
    _nq.pop();
    _hm.erase(np -> coord);
    return np;
}

void Frontier::push(Coordinate const& coord, Node::Ptr const& parent, double rank) {
    if(_hm.count(coord) == 0){
        auto np = std::make_shared<Node>(coord);
        np -> rank = rank;
        np -> parent = parent;
        auto handle = _nq.push(np);
        _hm[coord] = handle;
    } else {
        auto handle = _hm[coord];
        auto prev = *handle;
        if(prev->rank > rank){
            prev->rank = rank;
            prev->coord = coord;
            prev -> parent = parent;
            _nq.update(handle, prev);
        }
    }
}

bool Frontier::empty() {
    return _nq.empty();
}

// ClosedSet definition
bool ClosedSet::is_visited(Coordinate const& coord){
    return _nmap.count(coord) > 0;
}

void ClosedSet::set_visited(Coordinate const& coord){
    _nmap.insert(coord);
}

int ClosedSet::size(){
    return _nmap.size();   
}

void ClosedSet::save(std::string path) {
    std::ofstream out;
    out.open(path.c_str());
    for(auto it = _nmap.begin(); it != _nmap.end(); it++){
        auto record = *it;
        auto coord = record.position();
        out << coord.x() << " " << coord.y() << " " << record.heading() << std::endl;
    }
    out.close();
}