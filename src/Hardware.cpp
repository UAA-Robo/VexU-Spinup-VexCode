#include "Hardware.h"

Hardware::Hardware() {
    opticalSensor.setLight(vex::ledState::on);
    opticalSensor.setLightPower(100);
    wheelLeftFront.setBrake(vex::brakeType::brake);
    wheelLeftBack.setBrake(vex::brakeType::brake);
    wheelRightFront.setBrake(vex::brakeType::brake);
    wheelRightBack.setBrake(vex::brakeType::brake);

    leftWheels.setStopping(vex::brakeType::brake);
    rightWheels.setStopping(vex::brakeType::brake);

    driveTrain.setStopping(vex::brakeType::brake);
    smartDriveTrain.setStopping(vex::brakeType::brake);

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
