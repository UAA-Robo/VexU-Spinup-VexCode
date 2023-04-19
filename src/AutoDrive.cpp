#include "AutoDrive.h"

AutoDrive::AutoDrive(Hardware* hardware, RobotConfig* robotConfig, Telemetry* telemetry) {
    hw = hardware;
    rc = robotConfig;
    tm = telemetry;
    mp = new Map();
}

void AutoDrive::drive() {
    double const DRIVEVELPERCENT = 50;
    double flywheelVoltPercent = 70;
    //{-61.5, 35.5};
    std::pair<double,double> initPosition = tm->getGPSPosition();

    //tm->setInertiaHeadingToGPS(); 
    tm->setInertiaHeadingToGPS();
    tm->setManualPosition(initPosition); //TEST

    //Set bot at rollers and spin intake reveerse to get them
    //spinIntake(false, true);
    vex::wait(1000, vex::msec);
    
    //Drive backward and shoot 2 diskds
    rotateAndDriveToPosition({initPosition.first + 3.8, initPosition.second}, true);
    rotateAndDriveToPosition({initPosition.first, initPosition.second}, true);
    //moveDriveTrainDistance({DRIVEVELPERCENT, 0}, -3.8);
    
    /*
    rotateAndShoot(mp->mapElements.at(43), flywheelVoltPercent, 2);

    
    //Spin intake to pick up disks
    spinIntake(); 

    //Pick up 3 disks and move a bit past last one
    rotateAndDriveToPosition(mp->mapElements.at(29));
    moveDriveTrainDistance({DRIVEVELPERCENT, 0}, 4);

    //Shoot 3 disks
    rotateAndShoot(mp->mapElements.at(43), flywheelVoltPercent, 2);

    //Pick up 3 disks and move a bit past last one
    rotateAndDriveToPosition(mp->mapElements.at(22));
    moveDriveTrainDistance({DRIVEVELPERCENT, 0}, 4);

    //Shoot 3 disks
    rotateAndShoot(mp->mapElements.at(43), flywheelVoltPercent, 3);
    */

}

void AutoDrive::shootAtDesiredVelocity(double velocityPercent, int numFlicks)
{   
    double desiredVoltage = velocityPercent / 100 * 12000;
    for(int i = 0; i < numFlicks; ++i)
    {
        spinFlywheel(desiredVoltage);
        vex::wait(1500, vex::msec);
        while(hw->flywheel.velocity(vex::percent) < velocityPercent);

        flickDisk();
        vex::wait(500, vex::msec);
    }
}

void AutoDrive::rotateToHeading(double heading) {
    const double DEADBAND = 1;
    const double ROTATIONVEL = 20;
    int directionMultiplier = 1;

    //Corrects heading to be from 0-360 from the x axis counterclockwise
    heading  = fmod((fmod(heading,360) + 360), 360); //fmod is modulo on doubles: heading = (heading % 360 + 360) % 360

    //Determines whether to rotate left or right based on the  shortest distance
    double differenceInHeading = heading - tm->getInertiaHeading();
    if (360 - (differenceInHeading) < differenceInHeading) directionMultiplier = -1; 
    
    while (fabs(heading - tm->getInertiaHeading()) > DEADBAND) {
        //directionMultiplier == 1: rotate at to the left at ROTATIONVEL%. directionMultiplier == -1: rotate at to the right at ROTATIONVEL%.
        moveDriveTrain({0, -ROTATIONVEL * directionMultiplier}); 
    }   
    hw->driveTrain.stop();
}

void AutoDrive::rotateToPosition(std::pair<double,double> finalPosition, bool ISBACKROTATION, bool ISUSINGGPS)
{
    std::pair<double,double> currPos = tm->getManualPosition();
    if (ISUSINGGPS) currPos = tm->getGPSPosition();
    double heading = tm->getHeadingBtwnPoints(currPos, finalPosition);

    if(ISBACKROTATION) heading -= 180;
    rotateToHeading(heading);

}

void AutoDrive::rotateToPosition(GameElement* gameElement, bool ISUSINGGPS) {
    std::pair<double,double> currPos = tm->getManualPosition();
    if (ISUSINGGPS) currPos = tm->getGPSPosition();
    double heading = tm->getHeadingBtwnPoints(currPos, gameElement->GetPositionWithMinOffset());

    if(gameElement->GetAlignment()) heading -= 180;
    rotateToHeading(heading);
    
}

void AutoDrive::rotateAndDriveToPosition(GameElement* element, bool ISUSINGGPS) {
    std::pair<double,double> currPos = tm->getManualPosition();
    if (ISUSINGGPS) currPos = tm->getGPSPosition();

    std::pair<double, double> position = element->GetPositionWithMinOffset();

    rotateToPosition(element, ISUSINGGPS); 
    double distanceToPosition = tm->getDistanceBtwnPoints(currPos, position); //inches
    
    moveDriveTrainDistance({70, 0}, distanceToPosition);

    tm->setManualPosition(position);
}

void AutoDrive::rotateAndDriveToPosition(std::pair<double,double> position, bool ISBACKTOPOSITION, bool ISUSINGGPS) {
    std::pair<double,double> currPos = tm->getManualPosition();
    if (ISUSINGGPS) currPos = tm->getGPSPosition();

    rotateToPosition(position, ISBACKTOPOSITION); 

    double distanceToPosition = tm->getDistanceBtwnPoints(currPos, position); //inches
    
    moveDriveTrainDistance({70, 0}, distanceToPosition); //Drive at 70% velocity
    tm->setManualPosition(position);
}

void AutoDrive::rotateAndShoot(GameElement* goal, double velocityPercent, int numDisksToShoot, bool ISUSINGGPS) {
    
    rotateToPosition(goal, ISUSINGGPS);   //Red Goal
    shootAtDesiredVelocity(velocityPercent, numDisksToShoot);
}