#ifndef HLUT_H_
#define HLUT_H_

#include <boost/functional/hash.hpp>
#include <boost/unordered_map.hpp>

#include "graph.h"

int hlut_hdg_idx(double heading) {
    return Constants::heading_index(heading, Constants::hlut_hdg);
}

class Query{
public:
    Query() {};

    Query(double dx, double dy, double init_hdg, double goal_hdg, double cost=0)
        : _dx{dx}, _dy{dy}, _init_hdg{init_hdg}, _goal_hdg{goal_hdg}, _cost{cost} {};

    Query(const Coordinate& coord)
        : _dx{coord.position().x()}, _dy{coord.position().y()}, _goal_hdg{coord.bearing()} {};

    Vector2_i cell_idx() const {
        return Vector2_i(
            static_cast<int>(std::trunc(_dx/Constants::hlut_size)),
            static_cast<int>(std::trunc(_dy/Constants::hlut_size))
        );
    }

    void rotate(double wind_dir){
        // Rotate heading to wind coordinate frame
        _init_hdg = wrap_heading_360(_init_hdg - wind_dir);
        _goal_hdg = wrap_heading_360(_goal_hdg - wind_dir);

        // Rotate coordinates to wind coordinate frame
        double dx = _dx, dy = _dy;
        double rot = to_rad(-wind_dir);
        _dx = dx*cos(rot) - dy*sin(rot);
        _dy = dx*sin(rot) + dy*cos(rot);

        // Handle heading > 180
        if(_init_hdg > 180){
            _init_hdg = wrap_heading_360(360 - _init_hdg);
            _goal_hdg = wrap_heading_360(360 - _goal_hdg);
            _dy = -_dy;
        }
    }

    // Project query to land inside a square with given size,
    // return offset distance
    bool project(double max_dist, double& p_dist){
        // Projection invalid
        if(false && std::abs(_dx) >= max_dist && std::abs(_dy) >= max_dist){
            p_dist = wind_corrected_distance(
                Vector2_d(0, 0),
                Vector2_d(_dx, _dy)
            );
            return false;
        }
        double dx = _dx, dy = _dy;
        _dx = std::max(-max_dist, std::min(dx, max_dist));
        _dy = std::max(-max_dist, std::min(dy, max_dist));

        // Projected distance compensated for wind speed
        p_dist = wind_corrected_distance(
            Vector2_d(_dx, _dy),
            Vector2_d(dx, dy)
        );
        return true;
    }

    double cost() const { return _cost; };

    friend std::size_t hash_value(const Query& q) {
        std::size_t seed = 0;
        Vector2_i idx = q.cell_idx();
        boost::hash_combine(seed, idx[0]);
        boost::hash_combine(seed, idx[1]);
        boost::hash_combine(seed, hlut_hdg_idx(q._init_hdg));
        boost::hash_combine(seed, graph_hdg_idx(q._goal_hdg));
        return seed;       
    }

    friend std::ostream& operator<<(std::ostream& os, const Query& q){
        os << q._dx << " " << q._dy << " " << q._init_hdg << " " << q._goal_hdg << " " << q._cost;
        return os;
    }

    friend std::istream& operator>>(std::istream& is, Query& q){
        is >> q._dx >> q._dy >> q._init_hdg >> q._goal_hdg >> q._cost;
        return is;
    }

    bool operator==(const Query& other) const {
        return cell_idx() == other.cell_idx() &&
               (hlut_hdg_idx(_init_hdg) == hlut_hdg_idx(other._init_hdg)) &&
               (graph_hdg_idx(_goal_hdg) == graph_hdg_idx(other._goal_hdg));
    }
private:
    double _dx{0};
    double _dy{0};
    double _init_hdg{0};
    double _goal_hdg{0};
    double _cost{0};
};

class HLUT{
public:
    HLUT(
        double wind_spd=Constants::wind_spd(),
        double wind_dir=Constants::wind_dir(),
        double max_size=Constants::hlut_outer(),
        double min_size=Constants::hlut_inner()
    ) : _max_size{max_size},
        _min_size{min_size},
        _wind_dir{wind_dir},
        _wind_spd{wind_spd} {};

    void generate(MotionPrimitiveSet& primitives);
    void fill_empty(MotionPrimitiveSet& primitives, double start_hdg);

    bool lookup_cost(
        const Coordinate& start,
        const Coordinate& goal,
        double& cost,
        bool project=true
    ) const;

    void load_binary(std::string base_path);
    void save_binary(std::string base_path);
    void save_visual(std::string path, double start_hdg, double goal_hdg);
    int size(){return lookup.size();};
private:
    void dijkstra_search(MotionPrimitiveSet& primitives, double start_hdg);
    void astar_search(MotionPrimitiveSet& primitives, double start_hdg, const Coordinate& goal);
    void save_cost(const Coordinate& coord, double start_hdg, double cost);
    bool exists(const Coordinate& coord, double start_hdg);
    bool in_allowed_area(const Coordinate& coord);

    double _max_size{0};
    double _min_size{0};
    double _wind_dir{0};
    double _wind_spd{0};
    boost::unordered_set<Query> lookup;
};

#endif // HLUT_H_