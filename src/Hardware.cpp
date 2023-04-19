#include "Hardware.h"

Hardware::Hardware() {
     wheelLeftFront.setBrake(vex::brakeType::brake);
     wheelLeftBack.setBrake(vex::brakeType::brake);
     wheelRightFront.setBrake(vex::brakeType::brake);
     wheelRightBack.setBrake(vex::brakeType::brake);

    leftWheels.setStopping(vex::brakeType::brake);
    rightWheels.setStopping(vex::brakeType::brake);

    driveTrain.setStopping(vex::brakeType::brake);
    smartDriveTrain.setStopping(vex::brakeType::brake);
}