#include "okapi/api.hpp"
using namespace okapi;
namespace base_functions{
    //TODO ask if this structure is good
    class Arm{
        public:
            Motor left;
            Motor right;
/*
            Arm(u_int8_t leftPort, u_int8_t rightPort){
                left = Motor(leftPort,true,AbstractMotor::gearset::red,AbstractMotor::encoderUnits::rotations);
                right = Motor(rightPort,false,AbstractMotor::gearset::red,AbstractMotor::encoderUnits::rotations);
            };*/




    };

    std::shared_ptr<ChassisController> initMobileBase(u_int8_t frontLeft,
                                                    u_int8_t frontRight,
                                                    u_int8_t backLeft,
                                                    u_int8_t backRight,
                                                    QLength wheelDiameter,
                                                    QLength wheelTrack){
        // Generate the motors
        Motor motorLB = Motor(backLeft,true,AbstractMotor::gearset::green,AbstractMotor::encoderUnits::rotations);
        Motor motorLF = Motor(frontLeft,false,AbstractMotor::gearset::green,AbstractMotor::encoderUnits::rotations);
        Motor motorRB = Motor(backRight,false,AbstractMotor::gearset::green,AbstractMotor::encoderUnits::rotations);
        Motor motorRF = Motor(frontRight,true,AbstractMotor::gearset::green,AbstractMotor::encoderUnits::rotations);
        const std::initializer_list<Motor> left = {motorLB, motorLF};
        const std::initializer_list<Motor> right = {motorRB, motorRF};

        // Create the chassis with the two motor groups
        std::shared_ptr<ChassisController> drive =
            ChassisControllerBuilder().withMotors(left,right)
            .withDimensions(AbstractMotor::gearset::green,{{wheelDiameter,wheelTrack},imev5GreenTPR})
            .build();
        return drive;

    }

}
