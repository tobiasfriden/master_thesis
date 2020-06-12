#include "motion_primitive.h"

int main(int argc, char** argv){
    MotionPrimitiveSet ms;
    if(atoi(argv[1])){
        ms.generate(true);
        ms.save_to_file("../primitives/");
    } else {
        ms.load_from_file("../primitives/");
        ms.save_visual("../primitives/visual.txt", atoi(argv[2]), atoi(argv[3]), 1);
        ms.save_visual("../primitives/visual_low.txt", atoi(argv[2]), atoi(argv[3]), 1-Constants::wind_error());
        ms.save_visual("../primitives/visual_high.txt", atoi(argv[2]), atoi(argv[3]), 1+Constants::wind_error());
    }
}