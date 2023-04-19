#include "Robot.h"

Robot::Robot()
{
    hw = new Hardware();
    rc = new RobotConfig(hw);
    tm = new Telemetry(hw, rc);
    
    //userDrive = new UserDrive(hw, rc, tm); 
    autoDrive = new AutoDrive(hw, rc, tm);
}

/*
void Robot::drive() {
    userDrive->drive();
}
*/

void Robot::driveAuto() {
    autoDrive->drive();
}

/*
void Robot::printDistToGoal(){
    while(1){
        vex::wait(500,vex::msec);
        hw->master.clear();
        vex::wait(100,vex::msec)
        //hw->master.print(0, 0, "Distance is %.0f", userDrive->outputDistanceToGoal());
        hw->master.print(0, 0, "x: %.3f y: %.3f", tm->getGPSPosition().first, tm->getGPSPosition().second);
        vex::wait(70,vex::msec)
    }
}

*/