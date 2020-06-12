#include "waypoint_opt.h"

std::vector<Coordinate> WaypointOpt::improve_path(
    Simulator& sim,
    const Obstacles& obst,
    const std::vector<Coordinate>& path
){
    std::vector<Vector2_d> mission;
    std::transform(
        path.begin(),
        path.end(),
        std::back_inserter(mission),
        [](Coordinate c) -> Vector2_d {
            return c.waypoint();
        }
    );

    double h = 0.3;
    double rel_tol = 1e-5;
    double abs_tol = 0.005;
    double max_iter = 100;

    bool back_stepping_line_search = true;
    bool converged = false;
    int iter = 1;

    _terminal_scale = 1;
    double max_terminal_scale = 1;

    std::cout << "init WP:s :" << std::endl;
    for(int i = 0; i<mission.size(); i++){
        std::cout << i << "]" << "(x,y) = (" << mission[i].x()  << "," << mission[i].y()  << ")" << std::endl;
    }

    double J_iter, J_init;
    J_iter = get_mission_J(sim, obst, mission);
    J_init = J_iter;
    std::cout << "Current cost: " << J_iter << std::endl;

    while(!converged && iter < max_iter){
        int N_WP = int(mission.size());
        int M = 2;

        std::cout << "Iteration: " << iter << std::endl;
        iter++;

        // Compute gradient:
        Eigen::VectorXd GradJ(N_WP*M);
        GradJ.setZero();
        this->compute_gradient(sim, obst, GradJ, mission, h);

        //GradJ.normalize();
        if(GradJ.norm() < abs_tol){
            std::cout << "Locally optimal point: |grad J| < abs_tol" << std::endl;
            if(_terminal_scale < max_terminal_scale){
                _terminal_scale*=10;
            } else {
                converged = true;
                break;
            }
        }

        Eigen::VectorXd stepDirection(N_WP*M);
        stepDirection = - GradJ.normalized();

        double step_size = 50.0;

        // Backstepping line-search:
        double tau = 0.5; double c = 0.5;
        double t = - c * GradJ.transpose() * stepDirection;

        std::vector<Vector2_d> controlPoints_temp = mission;
        for(int i = 1; i < mission.size()-1; i++){
            controlPoints_temp[i].x(controlPoints_temp[i].x() + step_size * stepDirection[i * 2]);
            controlPoints_temp[i].y(controlPoints_temp[i].y() + step_size * stepDirection[i * 2 + 1]);
        }
        double DeltaJ = J_iter - get_mission_J(sim, obst, controlPoints_temp);

        while (DeltaJ <= step_size * t && step_size > 0.01 && back_stepping_line_search){
            step_size *= tau;
            //std::cout << "step_size = " << step_size << std::endl;
            controlPoints_temp = mission;
            for(int i = 1; i < mission.size()-1; i++){
                controlPoints_temp[i].x(controlPoints_temp[i].x() + step_size * stepDirection[i * 2]);
                controlPoints_temp[i].y(controlPoints_temp[i].y() + step_size * stepDirection[i * 2 + 1]);
            }
            DeltaJ = J_iter - get_mission_J(sim, obst, controlPoints_temp);
            //std::cout << "DeltaJ = " << DeltaJ << std::endl;
        }

        std::cout << "DeltaJ / J_iter = " << DeltaJ/J_iter << std::endl;

        if(DeltaJ/J_iter < rel_tol){
            std::cout << "Locally optimal point: |delta J / J_iter| < rel_tol" << std::endl;
            J_iter = get_mission_J(sim, obst, mission);
            std::cout << "Final cost: " << J_iter << " Improvment: " << 100.0*(J_init- J_iter)/J_init << "%" << std::endl;
            if(_terminal_scale < max_terminal_scale){
                _terminal_scale*=10;
            } else {
                converged = true;
            }
        }

        mission = filter_mission(controlPoints_temp);
        J_iter = get_mission_J(sim, obst, mission);
    }

    std::vector<Coordinate> coords;
    std::transform(
        mission.begin(),
        mission.end(),
        std::back_inserter(coords),
        [](Vector2_d vec) -> Coordinate {
            return Coordinate(vec.x(), vec.y(), 0);
        }
    );

    return coords;
    
}

std::vector<Vector2_d> WaypointOpt::filter_mission(std::vector<Vector2_d> mission){
    bool points_ok = false;
    while(!points_ok){
        points_ok = true;
        std::vector<Vector2_d> filtered_mission;
        for(int i=0; i <= mission.size() - 1; i++){
            if(i==0 || i==mission.size()-1){
                filtered_mission.push_back(mission[i]);
                continue;
            }
            Vector2_d curr_point = mission[i];
            Vector2_d next_point = mission[i+1];
            double dist = (curr_point-next_point).Norm();
            if(dist <= 3*Constants::wp_r()){
                points_ok = false;
                std::cout << "filter point: " << dist << std::endl;
                Vector2_d mean_point(
                    (curr_point.x() + next_point.x())/2,
                    (curr_point.y() + next_point.y())/2
                );
                filtered_mission.push_back(mean_point);
                i += 1;
            } else {
                filtered_mission.push_back(curr_point);
            }
        }
        mission = filtered_mission;
    }
    return mission;
}

void WaypointOpt::compute_gradient(Simulator& sim, const Obstacles& obst, Eigen::VectorXd & GradJ, std::vector<Vector2_d> controlPoints, double h){
    // Using central differences:
    for(int i = 2; i< GradJ.size()-2; i++){
        std::vector<Vector2_d> controlPoints_temp_ph = controlPoints;
        std::vector<Vector2_d> controlPoints_temp_mh = controlPoints;
        if( i % 2 == 0){
            controlPoints_temp_ph[i/2].x(controlPoints_temp_ph[i/2].x() + h);
            controlPoints_temp_mh[i/2].x(controlPoints_temp_mh[i/2].x() - h);
        }else{
            controlPoints_temp_ph[i/2].y(controlPoints_temp_ph[i/2].y() + h);
            controlPoints_temp_mh[i/2].y(controlPoints_temp_mh[i/2].y() - h);
        }
        double J_ph = this->get_mission_J(sim, obst, controlPoints_temp_ph);
        double J_mh = this->get_mission_J(sim, obst, controlPoints_temp_mh);
        GradJ[i] = (J_ph - J_mh)/(2*h);
    }
}

double WaypointOpt::get_mission_J(Simulator& sim, const Obstacles& obst, const std::vector<Vector2_d>& mission){
    double J = 0;
    double J_obst = 0;
    double J_final = 0;
    double J_roll = 0;
    sim.reset(0, 0, _init_hdg, 0);
    for(int i=0; i<mission.size() - 1; i++){
        Vector2_d start = mission[i];
        Vector2_d end = mission[i+1];
        std::vector<Vector2_d> traj;
        J += sim.simulate_waypoints(start, end, traj, false);
        for(int j=0; j<traj.size(); j+= 50){
            double dist = obst.obstacle_dist(traj[j]);
            dist = std::max(Constants::safety_dist() - dist, 0.0);
            if(dist > 0){
                J_obst += Constants::obst_w()/dist*sim.dt();
            }
        }
    }
    double xtrack_error = std::abs(sim.xtrack_error());
    J_final += xtrack_error*Constants::xtrack_w();

    double hdg_error = std::abs(_goal_hdg - sim.path_bearing());
    hdg_error = std::abs(std::min(hdg_error, 360-hdg_error));
    J_final += hdg_error*Constants::hdg_w();
    J_final *= _terminal_scale;

    for(auto rate : sim.roll_rates()){
        J_roll += Constants::roll_rate_w()*std::pow(rate, 2)*sim.dt();
    }

    //std::cout << "J: " << J << " J_obst: " << J_obst << " J_final: " << J_final << " J_roll: " << J_roll << std::endl;

    return J + J_obst + J_roll + J_final;
}