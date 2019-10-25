#include "hlut.h"

void HLUT::generate(MotionPrimitiveSet& primitives){
    for(double hdg=0; hdg<=180; hdg += Constants::hlut_hdg){
        std::cout << "generating: " << hdg << std::endl;
        dijkstra_search(primitives, hdg);
    }
    for(double hdg = 0; hdg<=180; hdg += Constants::hlut_hdg){
        fill_empty(primitives, hdg);
    }
}

void HLUT::fill_empty(MotionPrimitiveSet& primitives, double start_hdg){
    int missing = 0;
    for(double x = -_min_size; x <= _min_size; x+= Constants::hlut_size){
        for(double y = -_min_size; y <= _min_size; y+= Constants::hlut_size){
            for(double hdg=0; hdg < 360; hdg += Constants::hlut_hdg){
                Coordinate coord(x, y, hdg);
                if(!exists(coord, start_hdg)){
                    missing++;
                    astar_search(primitives, start_hdg, coord);
                }
            }
        }
    }
    std::cout << "start hdg: " << start_hdg << " missing: " << missing << std::endl;
}

void HLUT::dijkstra_search(
    MotionPrimitiveSet& primitives,
    double start_hdg
){
    Frontier frontier;
    ClosedSet closed_set;

    Coordinate start(0, 0, start_hdg);
    frontier.push(start, nullptr, 0);

    while(!frontier.empty()){
        Node::Ptr current = frontier.top();

        // if(current -> coord == Coordinate(100, -100, 90)){
        //     std::cout << current -> rank << " " << current -> coord << std::endl;
        // }

        if(closed_set.is_visited(current -> coord)){
            std::cout << "already visited: " << current -> coord << std::endl;
        }

        auto coords = current -> coord.get_mp_neighbours(primitives, 0);
        for(auto c: coords){
            if(c == current -> coord){
                continue;
            }
            if(!closed_set.is_visited(c) && in_allowed_area(c)){
                frontier.push(c, current, c.cost());
            }
        }
        closed_set.set_visited(current -> coord);
        save_cost(current -> coord, start_hdg, current -> coord.cost());
    }
    std::cout << "generated " << lookup.size() << " entries" << std::endl;
    // Coordinate best_coord(0, 0, 0);
    // int missing = 0;
    // double min_norm = std::numeric_limits<double>().max();
    // for(double x = -_max_size; x <= _max_size; x+= hlut_size){
    //     for(double y = -_max_size; y <= _max_size; y+= hlut_size){
    //         for(double hdg=0; hdg < 360; hdg += hlut_hdg){
    //             Coordinate coord(x, y, hdg);
    //             if(!exists(coord, start_hdg)){
    //                 if(coord.position().Norm() < 10){
    //                     std::cout << coord << std::endl;
    //                 }
    //                 missing++;
    //                 if(coord.position().Norm() < min_norm) {
    //                     min_norm = coord.position().Norm();
    //                     best_coord = coord;
    //                 }
    //             }
    //         }
    //     }
    // }
    // std::cout << "missing: " << missing << std::endl;
    // std::cout << "min coord: " << best_coord << std::endl;
}

void HLUT::astar_search(MotionPrimitiveSet& primitives, double start_hdg, const Coordinate& goal){
    Frontier frontier;
    ClosedSet closed_set;

    Query goal_q(goal);

    Coordinate start(0, 0, start_hdg);
    frontier.push(start, nullptr, 0);

    while(!frontier.empty()){
        Node::Ptr current = frontier.top();
        if(Query(current -> coord) == goal_q){
            std::cout << "found: " << goal_q << " " << current->coord.cost() <<  std::endl;
            save_cost(current -> coord, start_hdg, current -> coord.cost());
            return;
        }

        auto coords = current -> coord.get_mp_neighbours(primitives, 0);
        for(auto c: coords){
            if(!closed_set.is_visited(c)){
                double cost;
                lookup_cost(c, goal, cost, false);
                double rank = c.cost() + cost;
                frontier.push(c, current, rank);
            }
        }
        closed_set.set_visited(current -> coord);
    }
}

bool HLUT::lookup_cost(
    const Coordinate& start,
    const Coordinate& goal,
    double& cost,
    bool project
) const {
    Query q(
        goal.position().x() - start.position().x(),
        goal.position().y() - start.position().y(),
        start.heading(),
        goal.heading()
    );
    q.rotate(_wind_dir);
    auto it = lookup.find(q);
    if(it != lookup.end()){
        cost = it -> cost();
        return true;
    }

    // Project query to smaller HLUT
    if(project){
        double p_dist = q.project(_min_size);
        it = lookup.find(q);
        if(it != lookup.end()){
            cost = p_dist + it->cost();
            return true;
        }
    }

    cost = wind_corrected_distance(
        rotate(start.position(), -_wind_dir),
        rotate(goal.position(), -_wind_dir)
    );
    return false;
}

void HLUT::save_cost(const Coordinate& coord, double start_hdg, double cost){
    Vector2_d pos = coord.position();
    Query q(pos.x(), pos.y(), start_hdg, coord.heading(), cost);
    auto it = lookup.find(q);
    if(it == lookup.end() || cost < it -> cost()){
        lookup.insert(q);
    }
}

bool HLUT::exists(const Coordinate& coord, double start_hdg){
    Vector2_d pos = coord.position();
    return lookup.count(Query(pos.x(), pos.y(), start_hdg, coord.heading())) > 0;
}

bool HLUT::in_allowed_area(const Coordinate& coord){
    Vector2_d pos = coord.position();
    return std::abs(pos.x()) <= _max_size && std::abs(pos.y()) <= _max_size;
}

void HLUT::save_to_file(std::string path){
    std::ofstream out;
    out.open(path);

    for(auto it = lookup.begin(); it != lookup.end(); it++){
        out << *it << std::endl;
    }

    out.close();
}

void HLUT::load_from_file(std::string path){
    std::ifstream in;
    in.open(path);

    std::string line;
    Query q;
    while(getline(in, line)){
        std::stringstream ss(line);
        ss >> q;
        lookup.insert(q);
    }

    in.close();
}

void HLUT::save_binary(std::string path){
    std::ofstream out;
    out.open(path.c_str(), ios::binary);
    for(auto it = lookup.begin(); it != lookup.end(); it++){
        Query q = *it;
        out.write((char*)&q, sizeof(Query));
    }
    out.close();
}

void HLUT::load_binary(std::string path){
    std::ifstream in;
    in.open(path.c_str(), ios::binary);
    while(!in.eof()){
        Query q;
        in.read((char*)&q, sizeof(Query));
        lookup.insert(q);
    }
    in.close();
}

void HLUT::save_visual(std::string path, double start_hdg, double goal_hdg){
    ofstream out;
    out.open(path);

    for(double x = -_max_size; x <= _max_size; x += Constants::hlut_size){
        for(double y = -_max_size; y <= _max_size; y += Constants::hlut_size){
            double cost;
            if(lookup_cost(Coordinate(0, 0, start_hdg), Coordinate(x, y, goal_hdg), cost)){
                out << cost << " ";
            } else {
                out << -1 << " ";
            }
        }
        out << std::endl;
    }
    out.close();
}
