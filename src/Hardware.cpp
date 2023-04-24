#include "Hardware.h"

Hardware::Hardware() {
    opticalSensor.setLight(vex::ledState::on);
    opticalSensor.setLightPower(100);
    wheelLeftFront.setBrake(vex::brakeType::hold);
    wheelLeftBack.setBrake(vex::brakeType::hold);
    wheelRightFront.setBrake(vex::brakeType::hold);
    wheelRightBack.setBrake(vex::brakeType::hold);

    leftWheels.setStopping(vex::brakeType::hold);
    rightWheels.setStopping(vex::brakeType::hold);

    driveTrain.setStopping(vex::brakeType::hold);
    smartDriveTrain.setStopping(vex::brakeType::hold);

    launcher.setStopping(vex::brakeType::brake);

    //Calibrate sensors and wait while calibrating
    gpsSensor.calibrate();
    while (gpsSensor.isCalibrating()); 

    inertiaSensor.calibrate();
    while(inertiaSensor.isCalibrating());

    opticalSensor.integrationTime(100);

}

void Hardware::startupCheck()
{
    //vex::devices::numberOf
}
