#include "obstacle.h"

Obstacles::Obstacles(const S2LatLng& origin, double safety_m, std::string path) : _origin{origin} {
    _safety_dist = S2Earth::ToChordAngle(util::units::Meters(safety_m));
    build_index(path);
    std::cout << "built index with size: " << index.num_shape_ids() << std::endl;

    edge_q = new S2ClosestEdgeQuery(&index);
    point_q = new S2ContainsPointQuery<MutableS2ShapeIndex>(&index);
}

bool Obstacles::in_collision(const Vector2_d& coord) const {
    S2LatLng ll = to_latlng(coord);
    return point_q -> Contains(ll.ToPoint());
}

bool Obstacles::in_collision(const Vector2_d& start, const Vector2_d& goal) const {
    S2LatLng ll1 = to_latlng(start);
    S2LatLng ll2 = to_latlng(goal);

    S2ClosestEdgeQuery::EdgeTarget target(ll1.ToPoint(), ll2.ToPoint());
    return edge_q -> IsDistanceLessOrEqual(&target, _safety_dist);
}

double Obstacles::obstacle_dist(const Vector2_d& position) const {
    S2ClosestEdgeQuery::PointTarget target(to_latlng(position).ToPoint());
    return S2Earth::ToMeters(edge_q -> GetDistance(&target));
}

S2LatLng Obstacles::to_latlng(const Vector2_d& pos) const {
    return offset(_origin, pos.x(), pos.y());
}

void Obstacles::save_to_file(const S2LatLng& origin, std::string path){
    std::ofstream out(path.c_str());
    for(int i=0; i<index.num_shape_ids(); i++){
        S2Shape* shape = index.shape(i);
        for(int j=0; j < shape->num_edges(); j++){
            auto edge = shape -> edge(j);
            S2LatLng ll(edge.v0);
            Vector2_d offset = get_distance_NE(origin, ll);
            out << offset.x() << "," << offset.y() << " ";
        }
        out << std::endl;
    }
}

geojson Obstacles::read_features(std::string path) {
    std::ifstream in(path.c_str());
    std::stringstream buffer;
    buffer << in.rdbuf();
    return parse(buffer.str());
}

void Obstacles::build_index(std::string path){
    geojson geo = read_features(path);
    const auto& features = geo.get<feature_collection>();
    std::vector<S2Point> p_vector;
    for(const auto& f: features){
        p_vector.clear();
        const auto& p = f.geometry.get<polygon>();
        const auto& ring = p.at(0);
        for(auto coord: ring){
            p_vector.push_back(
                S2LatLng::FromDegrees(coord.y, coord.x).ToPoint().Normalize()
            );
        }
        p_vector.pop_back();
        index.Add(absl::make_unique<S2Loop::Shape>(new S2Loop(p_vector)));
    }
}