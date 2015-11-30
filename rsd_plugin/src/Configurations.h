
#ifndef GLOBALS_H_
#define GLOBALS_H_
#include <rw/math.hpp>
using namespace rw::math;

namespace Configurations
{
    const Q ConfigInit = Q(6,0.0,0.0,0.0,0.0,0.0,0.0);
    const Q ConfigImgCapture = Q(6,1.420,0.679,0.360,0.699,-1.070,0.890);
    const Q ConfigDropoff = Q(6,0.125,1.557,-0.679,-0.002,-0.879,0.125);
    //const Q ConfigDropoff = Q(6,0.207,1.584,-0.738,0.000,-0.847,0.207); //OLD
}
#endif
