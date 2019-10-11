#ifndef ASTAR_H_
#define ASTAR_H_

#include <vector>
#include <iostream>
#include <memory>

#include <s2/s2latlng.h>
#include <s2/s2crossing_edge_query.h>
#include <s2/s2closest_edge_query.h>
#include <boost/functional/hash.hpp>
#include <boost/unordered_map.hpp>
#include <boost/heap/d_ary_heap.hpp>
#include <mapbox/geojson.hpp>
#include <mapbox/geojson_impl.hpp>
#include <mapbox/geometry.hpp>

#include "simulation.h"
#include "motion_primitive.h"

class Grid {
public:
    Grid(
        S2CrossingEdgeQuery* q,
        S2ClosestEdgeQuery* dq,
        int32 step
    ) : _query{q}, _dQuery{dq}, step{step}, _inc{
        {0, step},
        {0, -step},
        {step, 0},
        {-step, 0},
        {step, step},
        {-step, step},
        {-step, -step},
        {step, -step}
    } {};
    int32 step;

    bool isCrossing(S2LatLng const& ll1, S2LatLng const& ll2) const;
    std::vector<S2LatLng> getNeighbours(const S2LatLng& c) const;

private:
    S2CrossingEdgeQuery* _query = nullptr;
    S2ClosestEdgeQuery* _dQuery = nullptr;
    int32 _inc[8][2];
};


class Coordinate {
public:
    Coordinate(
        const S2LatLng waypoint,
        const S2LatLng latLng,
        float heading,
        float cost
    ) : _latLng{latLng}, _heading{heading}, _waypoint{waypoint}, _cost{cost} {};

    Coordinate(const Coordinate& other)
      : _latLng{other._latLng},
        _heading{other._heading}, 
        _waypoint{other._waypoint},
        _cost{other._cost} {
            std::copy(other._states.begin(), other._states.end(), std::back_inserter(_states));
        };

    std::vector<Coordinate> getNeighbours(
        Simulator& sim,
        MotionPrimitiveSet& primitives
    );

    void add_states(std::vector<S2LatLng> states);
    mapbox::geojson::feature_collection get_states();

    float heuristic(const Coordinate& to);
    bool isEqual(const Coordinate& to) const;

    const S2LatLng& latLng() const { return _latLng; };
    const S2LatLng waypoint() const { return _waypoint; };
    float heading() const { return _heading; };
    float cost() const { return _cost; };
    std::vector<S2LatLng>& states() { return _states; };

    bool operator==(const Coordinate& other) const {
        return isEqual(other);
    }

    void operator=(const Coordinate& other){
        _latLng=other._latLng;
        _waypoint=other._waypoint;
        _heading=other._heading;
        _cost=other._cost;
        add_states(other._states);
    }

    friend std::size_t hash_value(const Coordinate& c) {
        std::size_t seed = 0;
        S2LatLng ll = c.latLng();
        boost::hash_combine(seed, ll.lat().e5()/50);
        boost::hash_combine(seed, ll.lng().e5()/50);
        return seed;       
    }

private:
    S2LatLng _latLng;
    S2LatLng _waypoint;
    float _heading{0};
    float _cost{0};

    std::vector<S2LatLng> _states;
};

class Node {
public:
    typedef std::shared_ptr<Node> Ptr;
    Node(
        const Coordinate& c
    ) : coord{c}, cost{0}, rank{0}, open{false}, closed{false} {};

    Coordinate coord;
    float cost = 0;
    float rank = 0;
    Ptr parent;
    bool open = false;
    bool closed = false;

    bool operator==(const Node& other) const;

    friend std::size_t hash_value(const Node& n) {
        std::size_t seed = 0;
        S2LatLng ll = n.coord.latLng();
        boost::hash_combine(seed, ll.lat().e5()/50);
        boost::hash_combine(seed, ll.lng().e5()/50);
        return seed;
    }
};

bool operator<(const Node& n1, const Node&n2);

struct node_cmp {
    bool operator()(const Node::Ptr n1, const Node::Ptr n2) const {
        return n1->rank > n2->rank;
    }
};

class NodeQueue {
public:
    NodeQueue() {};

    const Node::Ptr top();
    void push(const Node::Ptr& np);
    void update(const Node::Ptr& np);
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

class NodeMap {
public:
    const Node::Ptr get(const Coordinate& c);
    void save(std::string output);
private:
    typedef boost::unordered::unordered_map<Coordinate, Node::Ptr> nmap;
    nmap _nmap;
};

std::vector<Coordinate> astar(Simulator& sim, MotionPrimitiveSet& primitives, const S2LatLng& from, const S2LatLng& to, float goal_hdg);

std::vector<Coordinate> filter_solution(std::vector<Coordinate> pathVec);
#endif