#pragma once
#include "okapi/api.hpp"
using namespace okapi;
using namespace std;

namespace base_functions{
        extern bool ring_mill_already_pressed;
        extern bool pneumatic_already_pressed;


        void activate_ring_mill(std::shared_ptr<Motor> ringMillMotor,bool button_pressed);
        void activate_pneumatic(std::shared_ptr<pros::ADIPort> pneumaticPort,bool button_pressed);
        void lower_harm(std::shared_ptr<AsyncVelControllerBuilder> vc,int port_base_gripper);
        //void opcontrol(auto controllerPID, double target);

        class MyTimer : public AbstractTimer {

		MyTimer(QTime init) : AbstractTimer(init){};

		QTime millis() const override{};

	};
}
