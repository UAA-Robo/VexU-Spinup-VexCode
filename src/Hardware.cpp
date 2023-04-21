#include "Hardware.h"

Hardware::Hardware() {
     wheelLeftFront.setBrake(vex::brakeType::hold);
     wheelLeftBack.setBrake(vex::brakeType::hold);
     wheelRightFront.setBrake(vex::brakeType::hold);
     wheelRightBack.setBrake(vex::brakeType::hold);

    leftWheels.setStopping(vex::brakeType::hold);
    rightWheels.setStopping(vex::brakeType::hold);

    driveTrain.setStopping(vex::brakeType::hold);
    smartDriveTrain.setStopping(vex::brakeType::hold);
}