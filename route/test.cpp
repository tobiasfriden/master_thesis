#include <iostream>

#include <boost/heap/d_ary_heap.hpp>
#include <boost/functional/hash.hpp>
#include <s2/s2latlng.h>

#include <astar.h>

int main() {
    NodeQueue pq;
    NodeMap nm;

    Coordinate c1(S2LatLng::FromDegrees(50.0, 11.0), nullptr);
    Node::Ptr n1 = nm.get(c1);
    n1 -> rank = 1;

    Coordinate c2(S2LatLng::FromDegrees(50.0, 12.0), nullptr);
    Node::Ptr n2 = nm.get(c2);
    n2 -> rank = 2;

    pq.push(n1);
    pq.push(n2);

    Node::Ptr n3 = pq.top();
    std::cout << n3 -> rank << std::endl;
    auto c = n3 -> coord;
    std::cout << c.latLng().ToStringInDegrees() << std::endl;

    n2 -> rank = 0;
    pq.update(n2);

    Node::Ptr n4 = pq.top();
    std::cout << n4 -> rank << std::endl;
    auto casd = n4 -> coord;
    std::cout << casd.latLng().ToStringInDegrees() << std::endl;

}