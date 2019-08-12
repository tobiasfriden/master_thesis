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
    explicit Coordinate(
        const S2LatLng latLng,
        const Grid* grid
    ) : _latLng{latLng}, _grid{grid} {};

    void getNeighbours(std::vector<Coordinate>& coordVector);
    int32 neighbourCost(const Coordinate& to);
    int32 heuristicCost(const Coordinate& to);
    bool isEqual(const Coordinate& to) const;

    const S2LatLng& latLng() const { return _latLng; };

    bool operator==(const Coordinate& other) const {
        return isEqual(other);
    }

    friend std::size_t hash_value(const Coordinate& c) {
        std::size_t seed = 0;
        S2LatLng ll = c.latLng();
        boost::hash_combine(seed, ll.lat().e6());
        boost::hash_combine(seed, ll.lng().e6());
        return seed;       
    }

private:
    const S2LatLng _latLng;
    const Grid* _grid; 
};

class Node {
public:
    typedef std::shared_ptr<Node> Ptr;
    Node(
        const Coordinate& c
    ) : coord{c}, cost{0}, rank{0}, open{false}, closed{false} {};

    Coordinate coord;
    int32 cost = 0;
    int32 rank = 0;
    Ptr parent;
    bool open = false;
    bool closed = false;

    bool operator==(const Node& other) const;

    friend std::size_t hash_value(const Node& n) {
        std::size_t seed = 0;
        S2LatLng ll = n.coord.latLng();
        boost::hash_combine(seed, ll.lat().e6());
        boost::hash_combine(seed, ll.lng().e6());
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

private:
    typedef boost::heap::d_ary_heap<
        Node::Ptr,
        boost::heap::arity<4>,
        boost::heap::mutable_<true>,
        boost::heap::compare<node_cmp>
    > nQueue;

    typedef nQueue::handle_type nHandle;

    nQueue _nq;
    std::map<std::size_t, nHandle> _hm;
    boost::hash<Node::Ptr> _nHash;
};

class NodeMap {
public:
    const Node::Ptr get(const Coordinate& c);
private:
    typedef boost::unordered::unordered_map<Coordinate, Node::Ptr> nmap;
    nmap _nmap;
};

std::vector<Coordinate> astar(const Grid* grid, const S2LatLng& from, const S2LatLng& to);

#endif