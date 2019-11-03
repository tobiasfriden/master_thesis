#include "landing.h"

void Landing::setup_area(){
    double rot_rad = to_rad(_rotation);
    _w_dir = Vector2_d(std::cos(rot_rad), std::sin(rot_rad));
    _h_dir = Vector2_d(-std::sin(rot_rad), std::cos(rot_rad));
    _center = _origin + _height/2*_h_dir + _width/2*_w_dir;

    double origin_hdg = bearing_to(_origin, _center);
    Vector2_d h_corner = _origin + _height*_h_dir;
    double h_corner_hdg = bearing_to(h_corner, _center);
    if(origin_hdg > h_corner_hdg){
        _h_hdg_interval = Vector2_d(h_corner_hdg, origin_hdg);
    } else {
        _h_hdg_interval = Vector2_d(origin_hdg, h_corner_hdg);
    }
    _includes_360 = false;
    if(_h_hdg_interval.y() - _h_hdg_interval.x() > 90){
        _includes_360 = true;
    }
}

bool Landing::is_height_collision(double hdg){
    double inv_hdg = wrap_heading_360(hdg+180);
    if(_includes_360){
        return (hdg < _h_hdg_interval[0] || hdg > _h_hdg_interval[1]) ||
               (inv_hdg < _h_hdg_interval[0] || inv_hdg > _h_hdg_interval[1]);
    } else {
        return (hdg > _h_hdg_interval[0] && hdg < _h_hdg_interval[1]) ||
               (inv_hdg > _h_hdg_interval[0] && inv_hdg < _h_hdg_interval[1]);       
    }
}

Landing::line Landing::hdg_line(double hdg){
    double hdg_rad = to_rad(hdg);
    return  line{
        _center,
        _center + Vector2_d(std::cos(hdg_rad), std::sin(hdg_rad))
    };
}

std::vector<Landing::line> Landing::height_lines(){
    return std::vector<line>{
        line{
            _origin,
            _origin + _height*_h_dir
        },
        line{
            _origin + _width*_w_dir,
            _origin + _width*_w_dir + _height*_h_dir
        }
    };
}

std::vector<Landing::line> Landing::width_lines(){
    return std::vector<line>{
        line{
            _origin,
            _origin + _width*_w_dir
        },
        line{
            _origin + _height*_h_dir,
            _origin + _height*_h_dir + _width*_w_dir
        }
    };
}

Vector2_d Landing::intersection(Landing::line l1, Landing::line l2){
    Vector2_d pa = l1[0], pb = l1[1], pc = l2[0], pd = l2[1];

    double a1 = pb.y() - pa.y();
    double b1 = pa.x() - pb.x();
    double c1 = a1*pa.x() + b1*pa.y();

    double a2 = pd.y() - pc.y();
    double b2 = pc.x() - pd.x();
    double c2 = a2*pc.x() + b2*pc.y();

    double det = a1*b2 - a2*b1;
    if(std::abs(det) < 1e-6){
        std::cout << "error: lines are parallel!";
        double max = std::numeric_limits<double>().max();
        return Vector2_d(max, max);
    }
    return Vector2_d(
        (b2*c1 - b1*c2)/det,
        (a1*c2 - a2*c1)/det
    );
}

double Landing::groundspeed(double hdg){
    double wind_dir_rad = to_rad(_wind_dir);
    double hdg_rad = to_rad(hdg);
    // double wca = wind_correction_angle(hdg, _airspeed, _wind_dir, _wind_spd);
    // double wca_rad = to_rad(wca);

    Vector2_d airspeed_vec = _airspeed*Vector2_d(std::cos(hdg_rad), std::sin(hdg_rad));
    Vector2_d wind_vec = _wind_spd*Vector2_d(std::cos(wind_dir_rad), std::sin(wind_dir_rad));
    std::cout << (airspeed_vec + wind_vec) << std::endl;
    return (airspeed_vec + wind_vec).Norm();
}

Landing::line Landing::intersection_points(double hdg){
    std::vector<line> l;
    if(is_height_collision(hdg)){
        l = height_lines();
    } else {
        l = width_lines();
    }
    line hdg_l = hdg_line(hdg);
    Vector2_d p1 = intersection(hdg_l, l[0]);
    Vector2_d p2 = intersection(hdg_l, l[1]);
    return line{p1, p2};
}

void Landing::save_to_file(std::string path){
    std::ofstream out(path.c_str());
    out << _origin.x() << " "
        << _origin.y() << " "
        << _width << " "
        << _height << " "
        << _rotation << std::endl;
}

double Landing::land_distance(double hdg, double alt){
    double v = groundspeed(hdg);
    double flare_dist = _flare_h*v/_flare_sink;
    double approach_dist = (alt - _flare_h)*v/_max_sink;
    return flare_dist + approach_dist;
}

Vector2_d Landing::min_approach_point(double hdg, double alt){
    return offset_bearing(_center, hdg+180, 3*land_distance(hdg, alt));
}

bool Landing::feasible_heading(const Obstacles& obst, double hdg, double safety_h, double alt){
    Vector2_d aim_point = min_approach_point(hdg, alt);
    if(obst.in_collision(_center, aim_point)){
        return false;
    }
    double v = groundspeed(hdg);
    double flare_dist = _flare_h*v/_flare_sink;
    double approach_dist = std::max((safety_h - _flare_h), 0.0)*v/_max_sink;
    line points = intersection_points(hdg);
    double max_dist = (points[0] - points[1]).Norm();
    return flare_dist + approach_dist < max_dist;
}

Landing::line Landing::optimize(double hdg, double safety_h, double alt){
    // Parameters for problem setup
    double v = groundspeed(hdg);
    double h_0 = alt - _flare_h;
    safety_h -= _flare_h;
    double R_f = _flare_h*v/_flare_sink;
    line points = intersection_points(hdg);
    double R_c = (points[0] - points[1]).Norm()/2;
    std::cout << 2*R_c << std::endl;
    
    casadi::Opti opti;
    casadi::MX R_a = opti.variable();
    casadi::MX R_b = opti.variable();

    opti.subject_to(R_a >= 0);
    opti.subject_to(R_b >= 0);

    casadi::MX vz = h_0*v/(R_a - R_b - R_f);
    opti.subject_to(vz <= _max_sink);
    opti.subject_to(vz >= 0);

    opti.set_initial(R_a, land_distance(hdg,alt) - R_f + R_c);
    opti.set_initial(R_b, R_c);

    casadi::MX cross_h = h_0 - (R_a - 2*R_c)*vz/v;
    opti.subject_to(cross_h >= safety_h);

    opti.minimize(R_a*R_a + 20*(R_b-R_c)*(R_b-R_c));

    casadi::Dict plugin_opts;
    casadi::Dict solver_opts;
    solver_opts["print_level"] = 0;
    opti.solver("ipopt", plugin_opts, solver_opts);
    casadi::OptiSol sol = opti.solve();

    auto R_a_opt = sol.value(R_a);
    auto R_b_opt = sol.value(R_b);
    auto v_opt = sol.value(vz);
    auto cross = sol.value(cross_h);
    std::cout << "R_a-R_b: " << std::abs(static_cast<double>(R_a_opt)-static_cast<double>(R_b_opt)) << std::endl;
    std::cout << "R_b: " << std::abs(static_cast<double>(R_b_opt)-R_c) << std::endl;
    std::cout << "R_b: " << static_cast<double>(R_b_opt) << std::endl;
    std::cout << "cross: " << cross << std::endl;
    std::cout << "velocity: " << v << std::endl;
    
    Vector2_d start_pos = offset_bearing(_center, hdg, R_c);
    Vector2_d b_pos = offset_bearing(start_pos, hdg+180, static_cast<double>(R_b_opt));
    Vector2_d a_pos = offset_bearing(start_pos, hdg+180, static_cast<double>(R_a_opt) + 50);
    std::ofstream out("../land_opt.txt");
    out << b_pos.x() << " " << b_pos.y() << std::endl;
    out << a_pos.x() << " " << a_pos.y() << std::endl;

    return line{a_pos, b_pos};
}