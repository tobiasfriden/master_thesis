#ifndef _MOTION_PRIMITIVE_H_
#define _MOTION_PRIMITIVE_H_

#include <boost/functional/hash.hpp>
#include <boost/unordered_set.hpp>

#include "opt.h"
#include "constants.h"

class MotionPrimitive {
public:
    MotionPrimitive() {};

    MotionPrimitive(int wind_dir) : wind_dir_{wind_dir} {};

    MotionPrimitive(
        double north,
        double east,
        double heading,
        double wp_north,
        double wp_east,
        double cost,
        int goal_hdg,
        int wind_dir
    ) : north_{north},
        east_{east},
        heading_{fmod(heading, 360)},
        wp_north_{wp_north},
        wp_east_{wp_east},
        cost_{cost},
        goal_hdg_{goal_hdg},
        wind_dir_{wind_dir}
        {
            if(heading_ < 0) heading_ += 360;
        };

    MotionPrimitive(const MotionPrimitive& other)
        : north_{other.north()},
          east_{other.east()},
          heading_{other.heading()},
          wp_north_{other.wp_north()},
          wp_east_{other.wp_east()},
          cost_{other.cost()},
          goal_hdg_{other.goal_hdg()},
          wind_dir_{other.wind_dir()} {
              if(heading_ < 0) heading_ += 360;
          };

    friend std::size_t hash_value(const MotionPrimitive& mp){
        std::size_t seed=0;
        boost::hash_combine(seed, mp.goal_hdg());
        boost::hash_combine(seed, mp.wind_dir());
        return seed;
    }

    friend std::istream& operator>>(std::istream& is, MotionPrimitive& mp){
        is >> mp.north_
           >> mp.east_
           >> mp.heading_
           >> mp.wp_north_
           >> mp.wp_east_
           >> mp.cost_
           >> mp.goal_hdg_
           >> mp.wind_dir_;
        return is;
    }

    friend std::ostream& operator<<(std::ostream& os, const MotionPrimitive& mp){
        os << mp.north_ << " "
           << mp.east_ << " "
           << mp.heading_ << " "
           << mp.wp_north_ << " "
           << mp.wp_east_ << " "
           << mp.cost_ << " "
           << mp.goal_hdg_ << " "
           << mp.wind_dir_ << std::endl;
    }

    bool operator==(const MotionPrimitive& other) const {
        return goal_hdg_== other.goal_hdg() && wind_dir_ == other.wind_dir();
    }

    double north() const { return north_; };
    double east() const { return east_; };
    double heading() const { return heading_; };
    double wp_north() const { return wp_north_; };
    double wp_east() const { return wp_east_; };
    double cost() const { return cost_; };
    int goal_hdg() const { return goal_hdg_; };
    int wind_dir() const { return wind_dir_; }

    void set_goal_hdg(int goal_hdg){ goal_hdg_ = goal_hdg; };
    void set_wind_dir(int wind_dir){ wind_dir_ = wind_dir; };

private:
    double north_{0};
    double east_{0};
    double heading_{0};
    double wp_north_{0};
    double wp_east_{0};
    double cost_{0};
    int goal_hdg_{0};
    int wind_dir_{0};
};

class MotionPrimitiveSet{
public:
    MotionPrimitiveSet(double wind_spd=Constants::wind_spd()) : wind_spd_{wind_spd} {};

    void generate(bool log=false);
    void save_to_file(std::string output);
    void load_from_file(std::string input);

    void save(const MotionPrimitive& mp);
    
    std::vector<Vector2_d> get_expansions(double heading, int wind_dir) const;
    std::vector<MotionPrimitive> get_mp_expansions(double heading, int wind_dir) const;

private:
    int closest_wind_dir(double heading_diff) const;
    bool lookup(MotionPrimitive& mp) const;
    Vector2_d offset_vector(double offset_heading, double d_north, double d_east) const;

    double wind_spd_{0};
    boost::unordered_set<MotionPrimitive> mp_set;
};


#endif