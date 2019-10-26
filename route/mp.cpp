#include "motion_primitive.h"

int main(int argc, char** argv){
    MotionPrimitiveSet ms;
    if(atoi(argv[1])){
        ms.generate();
        ms.save_to_file("../primitives/");
    } else {
        ms.load_from_file("../primitives/");
    }
    auto exp = ms.get_mp_expansions(0, 0);
    std::cout << exp.size() << std::endl;
}