#ifndef _MOTION_PRIMITIVE_H_
#define _MOTION_PRIMITIVE_H_

#include <boost/functional/hash.hpp>
#include <boost/unordered_set.hpp>

#include "opt.h"

class MotionPrimitive {
public:
    MotionPrimitive(float north, float east, int goal_hdg, int wind_dir)
        : north_{north}, east_{east}, goal_hdg_{goal_hdg}, wind_dir_{wind_dir} {};

    MotionPrimitive(const MotionPrimitive& other)
        : north_{other.north()}, east_{other.east()}, goal_hdg_{other.goal_hdg()}, wind_dir_{other.wind_dir()} {};

    friend std::size_t hash_value(const MotionPrimitive& mp){
        std::size_t seed=0;
        boost::hash_combine(seed, mp.goal_hdg());
        boost::hash_combine(seed, mp.wind_dir());
        return seed;
    }

    bool operator==(const MotionPrimitive& other) const {
        return goal_hdg_== other.goal_hdg() && wind_dir_ == other.wind_dir();
    }

    float north() const { return north_; };
    float east() const { return east_; };
    int goal_hdg() const { return goal_hdg_; };
    int wind_dir() const { return wind_dir_; }

    void set_goal_hdg(int goal_hdg){ goal_hdg_ = goal_hdg; };
    void set_wind_dir(int wind_dir){ wind_dir_ = wind_dir; };

private:
    float north_{0};
    float east_{0};
    int goal_hdg_;
    int wind_dir_;
};

std::ostream& operator<<(std::ostream& out, const MotionPrimitive& mp) {
    out << "(" << mp.north() << "," << mp.east() << "," << mp.goal_hdg() << "," << mp.wind_dir() << ")\n";
    return out;
}

class MotionPrimitiveSet{
public:
    MotionPrimitiveSet(float wind_spd) : wind_spd_{wind_spd} {};

    void generate(bool log=false);
    void save_to_file(std::string output);
    void load_from_file(std::string input);

    void save(float north, float east, int goal_hdg, int wind_dir);
    
    std::vector<Vector2_f> get_expansions(float heading, int wind_dir);

private:
    int closest_wind_dir(float heading_diff);
    bool lookup(MotionPrimitive& mp);
    Vector2_f offset_vector(float offset_heading, float d_north, float d_east);

    float wind_spd_{0};
    boost::unordered_set<MotionPrimitive> mp_set;

    float wind_directions[16]{
        0,
        to_deg(atanf(0.5)),
        to_deg(atanf(1)),
        to_deg(atanf(2)),
        to_deg(M_PI/2),
        to_deg(M_PI/2+atanf(0.5)),
        to_deg(M_PI/2+atanf(1)),
        to_deg(M_PI/2+atanf(2)),
        to_deg(M_PI),
        to_deg(M_PI+atanf(0.5)),
        to_deg(M_PI+atanf(1)),
        to_deg(M_PI+atanf(2)),
        to_deg(1.5*M_PI),
        to_deg(1.5*M_PI+atanf(0.5)),
        to_deg(1.5*M_PI+atanf(1)),
        to_deg(1.5*M_PI+atanf(2)),        
    };
};


#endif