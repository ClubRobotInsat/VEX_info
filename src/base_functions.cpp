#include "okapi/api.hpp"
using namespace okapi;
namespace base_functions
{

	// Global variables
	bool ring_mill_already_pressed = false;
	bool pneumatic_already_pressed = false;

	// FIXME LOOPS IN TOO QUICK SO BUGGED CHANGE
	void activate_ring_mill(std::shared_ptr<Motor> ringMillMotor, bool button_pressed)
	{
		if (button_pressed)
		{
			if (ring_mill_already_pressed)
			{
				ringMillMotor->moveVelocity(0);
				ring_mill_already_pressed = false;
			}
			else
			{
				ringMillMotor->moveVelocity(200);
				ring_mill_already_pressed = true;
			}
		}
	}

	// FIXME LOOPS IN TOO QUICK SO BUGGED CHANGE
	void activate_pneumatic(std::shared_ptr<pros::ADIPort> pneumaticPort, bool button_pressed)
	{
		if (button_pressed)
		{
			if (pneumatic_already_pressed)
			{
				pneumaticPort->set_value(LOW);
				pneumatic_already_pressed = false;
			}
			else
			{
				pneumaticPort->set_value(HIGH);
				pneumatic_already_pressed = true;
			}
		}
	}

	// =================== Regarder en bas =========================

	void opcontrol(auto velController, double target) {
		// Execute the movement
		
	}

	void lower_harm(std::shared_ptr<AsyncVelControllerBuilder> vc,
					std::shared_ptr<Motor> m,
					int port_base_gripper) {
		TimeUtil t();
		const double kP = 1.0;
		const double kI = 0.001;
		const double kD = 0.1;
		const double target = 1.5;

		auto velControl = AsyncVelControllerBuilder()
					.withMotor(port_base_gripper)
					.withGains({kP, kI, kD})
					.build();

		opcontrol(velControl, target);
	}

	// S'inspirer de ce qu'ils ont fait pour les AsyncVelPIDController
	// Et librairies okapi
	// https://github.com/OkapiLib/OkapiLib/tree/master/test
}
