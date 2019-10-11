#include "motion_primitive.h"

int main(int argc, char** argv){
    MotionPrimitiveSet ms(7.5);
    if(atoi(argv[1])){
        ms.generate();
        ms.save_to_file("../primitives.txt");
    } else {
        ms.load_from_file("../primitives.txt");
    }
    auto exp = ms.get_expansions(30, 0);
    std::cout << exp.size() << std::endl;
}