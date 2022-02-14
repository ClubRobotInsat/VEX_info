#pragma once
#include "okapi/api.hpp"

using namespace okapi;

void activate_ring_mill(std::shared_ptr<Motor> ringMillMotor, bool button_pressed);
void activate_pneumatic(std::shared_ptr<pros::ADIPort> pneumaticPort, bool button_pressed);
void lower_harm(std::shared_ptr<AsyncVelControllerBuilder> vc, int port_base_gripper);
