#pragma once
#include "okapi/api.hpp"
using namespace okapi;
using namespace std;

namespace base_functions{
        extern bool button_last_state_ring_mill;
        extern bool button_last_state_pneumatic;
        extern bool ring_mill_last_state;
	extern bool pneumatic_last_state;



        void activate_ring_mill(std::shared_ptr<Motor> ringMillMotor,bool button_pressed);
        void activate_pneumatic(std::shared_ptr<pros::ADIPort> pneumaticPort,bool button_pressed);
}
