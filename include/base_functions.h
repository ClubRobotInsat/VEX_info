#pragma once
#include "okapi/api.hpp"
using namespace okapi;
using namespace std;

namespace base_functions{
        extern bool ring_mill_already_pressed;
        extern bool pneumatic_already_pressed;


        void activate_ring_mill(std::shared_ptr<Motor> ringMillMotor,bool button_pressed);
        void activate_penumatic(std::shared_ptr<pros::ADIPort> pneumaticPort,bool button_pressed);
}
