#include "Robot.h"

Robot::Robot()
{
    hw = new Hardware();
    rc = new RobotConfig(hw);
    tm = new Telemetry(hw, rc);
    
    userDrive = new UserDrive(hw, tm); 
    autoDrive = new AutoDrive(hw, rc, tm);
}


void Robot::drive() {
    //tm->printGPSInertiaData();
    userDrive->drive();
}


void Robot::driveAuto() {
    //tm->setInertiaHeadingToGPS();
    //while(1) tm->printGPSInertiaData();
    
    /*
    hw->wheelLeftBack.spinFor(1, vex::rotationUnits::rev, 40, vex::velocityUnits::pct, false);
    hw->wheelLeftFront.spinFor(1, vex::rotationUnits::rev, 40, vex::velocityUnits::pct, false);
    hw->wheelRightBack.spinFor(1, vex::rotationUnits::rev, 40, vex::velocityUnits::pct, false);
    hw->wheelRightFront.spinFor(1, vex::rotationUnits::rev, 40, vex::velocityUnits::pct);
    */
    autoDrive->drive();
}

