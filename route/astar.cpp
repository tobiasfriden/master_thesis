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
void Coordinate::getNeighbours(std::vector<Coordinate>& coordVector) {
    coordVector.clear();
    auto gridNeighbours = _grid -> getNeighbours(_latLng);
    for(auto const& ll : gridNeighbours) {
        coordVector.push_back(Coordinate(ll, _grid));
    }
}

int32 Coordinate::neighbourCost(const Coordinate& to) {
    return _latLng.GetDistance(to.latLng()).e6();
}

int32 Coordinate::heuristicCost(const Coordinate& to) {
    return _latLng.GetDistance(to.latLng()).e6();
}

bool Coordinate::isEqual(const Coordinate& to) const {
    const S2LatLng& ll = to.latLng();
    return _latLng.Normalized().GetDistance(to.latLng().Normalized()).e6() < 10000; //20*_grid->step;
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
    _hm.erase(_nHash(np));
    return np;
}

void NodeQueue::push(const Node::Ptr& np) {
    auto handle = _nq.push(np);
    auto hash = _nHash(np);
    _hm[hash] = handle;
}

void NodeQueue::update(const Node::Ptr& np) {  
    auto hash = _nHash(np);
    if(_hm.find(hash) != _hm.end()) {
        _nq.update(_hm[hash], np);
    } else {
        push(np);
    }
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

// A^* implementation
std::vector<Coordinate> astar(const Grid* grid, const S2LatLng& from, const S2LatLng& to){
    NodeMap nm;
    NodeQueue pq;
    std::vector<Coordinate> nbrVec;
    std::vector<Coordinate> pathVec;

    Node::Ptr fromNode = nm.get(Coordinate(from, grid));
    fromNode -> open = true;
    pq.push(fromNode);

    Node::Ptr toNode = nm.get(Coordinate(to, grid));

    while(true) {
        if(pq.empty()) {
            std::cout << "No path found" << std::endl;
            return pathVec;
        }

        Node::Ptr current = pq.top();
        current -> open = false;
        current -> closed = true;

        if(current -> coord.isEqual(toNode -> coord)) {
            std::cout << "Path found" << std::endl;
            Node::Ptr np = current;
            while(np != nullptr) {
                pathVec.push_back(np -> coord);
                np = np -> parent;
            }
            return pathVec;
        }

        current -> coord.getNeighbours(nbrVec);
        for(auto c: nbrVec) {
            int32 nbrCost = current -> coord.neighbourCost(c);
            int32 cost = current -> cost + nbrCost;
            auto nbrNode = nm.get(c);
            if(cost < nbrNode -> cost) {
                nbrNode -> open = false;
                nbrNode -> closed = false;
            }
            if(!(nbrNode -> open) && !(nbrNode -> closed)) {
                std::cout << c.latLng().ToStringInDegrees() << std::endl;
                std::cout << c.heuristicCost(toNode -> coord) << std::endl << std::endl;
                nbrNode -> cost = cost;
                nbrNode -> open = true;
                nbrNode -> rank = cost + c.heuristicCost(toNode -> coord);
                nbrNode -> parent = current;
                pq.update(nbrNode);
            }
        }
    }
}
 
