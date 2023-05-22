#include "Robot.h"

Robot::Robot()
{
    hw = new Hardware();
    rc = new RobotConfig(hw);
    tm = new Telemetry(hw, rc);
    
    
    userDrive = new UserDrive(hw, rc, tm); 
    autoDrive = new AutoDrive(hw, rc, tm);
}

void Robot::printFlywheelSpeed()
{
    hw->brain.Screen.print("%d", userDrive->getFlywheelSpeed());
}

void Robot::drive() {
    this->userDrive->drive();
}


void Robot::driveAuto() {
    autoDrive->drive();
}

