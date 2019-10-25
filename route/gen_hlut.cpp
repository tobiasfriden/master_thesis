#include <chrono>

#include "hlut.h"

int main(int argc, char** argv){
    HLUT hlut(atof(argv[1]));
    MotionPrimitiveSet primitives(0);
    primitives.load_from_file("../primitives.txt");
    if(atoi(argv[2])){
        auto start = std::chrono::system_clock::now();
        hlut.generate(primitives);
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> diff = end - start;
        std::cout << "generation time: " << diff.count() << std::endl;
        hlut.save_binary("../hlut.bin");
    } else {
        hlut.load_binary("../hlut.bin");
        hlut.save_visual("../hlut_viz.txt", 0, atof(argv[3]));
    }

    double cost;
    hlut.lookup_cost(
        Coordinate(10, 0, 270),
        Coordinate(0, -100, 180),
        cost
    );
    std::cout << cost << std::endl;

    hlut.lookup_cost(
        Coordinate(0, 0, 90),
        Coordinate(-10, 100, 180),
        cost
    );
    std::cout << cost << std::endl;
}