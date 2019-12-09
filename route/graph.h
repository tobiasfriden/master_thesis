#ifndef GRAPH_H_
#define GRAPH_H_

#include <vector>
#include <iostream>
#include <memory>

#include <boost/functional/hash.hpp>
#include <boost/unordered_map.hpp>
#include <boost/heap/d_ary_heap.hpp>
#include <mapbox/geojson.hpp>
#include <mapbox/geojson_impl.hpp>
#include <mapbox/geometry.hpp>

#include "simulation.h"
#include "motion_primitive.h"
#include "constants.h"
#include "obstacle.h"

int graph_hdg_idx(double heading){
    return Constants::heading_index(heading, Constants::hdg_size);
}

class Coordinate {
public:
    Coordinate(double north, double east, double heading)
        : _position{Vector2_d(north, east)},
          _waypoint{Vector2_d(north, east)},
          _heading{wrap_heading_360(heading)},
          _bearing{wrap_heading_360(heading)} {};

    Coordinate(
        const Vector2_d waypoint,
        const Vector2_d position,
        double heading,
        double bearing,
        double cost
    ) : _position{position},
        _heading{wrap_heading_360(heading)},
        _bearing{wrap_heading_360(bearing)},
        _waypoint{waypoint},
        _cost{cost} {};

    Coordinate(const Coordinate& other)
      : _position{other._position},
        _heading{other._heading},
        _bearing{other._bearing}, 
        _waypoint{other._waypoint},
        _cost{other._cost} {
            std::copy(other._states.begin(), other._states.end(), std::back_inserter(_states));
        };

    std::vector<Coordinate> get_neighbours(
        Simulator& sim,
        const MotionPrimitiveSet& primitives
    );

    std::vector<Coordinate> get_neighbours(
        Simulator& sim,
        const MotionPrimitiveSet& primitives,
        const Obstacles& obst
    );

    std::vector<Coordinate> get_mp_neighbours(
        const MotionPrimitiveSet& primitives,
        int wind_dir
    );

    std::vector<Coordinate> get_mp_neighbours(
        const MotionPrimitiveSet& primitives,
        const Obstacles& obst,
        int wind_dir
    );

    void add_states(std::vector<Vector2_d> states);
    mapbox::geojson::feature_collection get_states(S2LatLng const& origin);

    const Vector2_d& position() const { return _position; };
    const Vector2_d waypoint() const { return _waypoint; };
    double heading() const { return _heading; };
    double bearing() const { return _bearing; };
    double cost() const { return _cost; };
    std::vector<Vector2_d>& states() { return _states; };

    Vector2_i cell_idx() const {
        return Vector2_i(
            static_cast<int>(std::trunc(_position.x()/Constants::cell_size)),
            static_cast<int>(std::trunc(_position.y()/Constants::cell_size))
        );
    }

    bool operator==(const Coordinate& other) const {
        return cell_idx() == other.cell_idx() && graph_hdg_idx(_bearing) == graph_hdg_idx(other.bearing());
    }

    bool operator!=(const Coordinate& other) const {
        return !(*this==other);
    }

    void operator=(const Coordinate& other){
        _position=other._position;
        _waypoint=other._waypoint;
        _heading=other._heading;
        _bearing=other._bearing;
        _cost=other._cost;
        add_states(other._states);
    }

    friend std::size_t hash_value(const Coordinate& c) {
        std::size_t seed = 0;
        Vector2_i idx = c.cell_idx();
        boost::hash_combine(seed, idx.x());
        boost::hash_combine(seed, idx.y());
        boost::hash_combine(seed, graph_hdg_idx(c.bearing()));
        return seed;       
    }

private:
    Vector2_d _position;
    Vector2_d _waypoint;
    double _heading{0}; // UAV inertial frame heading
    double _bearing{0}; // UAV velocity vector direction
    double _cost{0};

    std::vector<Vector2_d> _states;
};

std::ostream& operator<<(std::ostream& os, const Coordinate& coord){
    os << "(" << coord.position().x() << "," << coord.position().y() << "," << coord.heading() << ")\n";
    return os;
}

class Node {
public:
    typedef std::shared_ptr<Node> Ptr;
    Node(
        const Coordinate& c
    ) : coord{c}, rank{0} {};

    Coordinate coord;
    double rank = 0;
    Ptr parent;

    boost::hash<Coordinate> c_hash;

    bool operator==(const Node& other) const {
        return coord == other.coord;
    }

    friend std::size_t hash_value(const Node& n) {
        return n.c_hash(n.coord);
    }
};

bool operator<(const Node& n1, const Node&n2){
    return n1.rank < n2.rank;
}

struct node_cmp {
    bool operator()(const Node::Ptr n1, const Node::Ptr n2) const {
        return n1->rank > n2->rank;
    }
};

class Frontier {
public:
    Frontier() {};

    const Node::Ptr top();
    void push(Coordinate const& coord, Node::Ptr const& parent, double rank);
    bool empty();
    int size() { return _nq.size(); };

private:
    typedef boost::heap::d_ary_heap<
        Node::Ptr,
        boost::heap::arity<4>,
        boost::heap::mutable_<true>,
        boost::heap::compare<node_cmp>
    > nQueue;

    typedef nQueue::handle_type nHandle;

    nQueue _nq;
    boost::unordered_map<Coordinate, nHandle> _hm;
};

class ClosedSet {
public:
    void set_visited(Coordinate const& coord);
    bool is_visited(Coordinate const& coord);
    void save(std::string output);
    int size();
private:
    boost::unordered_set<Coordinate> _nmap;
};


#endif