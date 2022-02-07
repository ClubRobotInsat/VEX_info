#pragma once
#include "okapi/api.hpp"
using namespace okapi;
using namespace std;

namespace base_functions{
        std::shared_ptr<ChassisController> initMobileBase(u_int8_t frontLeft,
                                                    u_int8_t frontRight,
                                                    u_int8_t backLeft,
                                                    u_int8_t backRight,
                                                    QLength wheelDiameter,
                                                    QLength wheelTrack);
}
