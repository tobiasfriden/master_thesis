#include "motion_primitive.h"

int main(int argc, char** argv){
    MotionPrimitiveSet ms;
    if(atoi(argv[1])){
        ms.generate(true);
        ms.save_to_file("../primitives.txt");
    } else {
        ms.load_from_file("../primitives.txt");
    }
    auto exp = ms.get_mp_expansions(0, 0);
    for(auto mp : exp){
        std::cout << mp;
    }
    std::cout << exp.size() << std::endl;
}