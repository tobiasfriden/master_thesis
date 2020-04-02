#include <chrono>

#include "hlut.h"

int main(int argc, char** argv){
    HLUT hlut;
    MotionPrimitiveSet primitives;
    primitives.load_from_file("../primitives/");
    if(atoi(argv[1])){
        auto start = std::chrono::system_clock::now();
        hlut.generate(primitives);
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> diff = end - start;
        std::cout << "generation time: " << diff.count() << std::endl;
        hlut.save_binary("../hlut/");
    } else {
        
        hlut.load_binary("../hlut/");
        std::cout << "hlut size: " << hlut.size() << std::endl;
        hlut.save_visual("../hlut_viz.txt", atof(argv[2]), atof(argv[3]));
    }

    double cost;

    hlut.lookup_cost(
        Coordinate(0, 0, 0),
        Coordinate(-500, -500, 0),
        cost
    );
    std::cout << cost << std::endl;
}