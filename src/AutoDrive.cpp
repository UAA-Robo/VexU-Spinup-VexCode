#include "AutoDrive.h"

AutoDrive::AutoDrive(Hardware* hardware, RobotConfig* robotConfig, Telemetry* telemetry): Drive(hardware, robotConfig, telemetry) {
    
}

void AutoDrive::drive() {
    //double const DRIVEVELPERCENT = 30;
    //double flywheelVoltPercent = 70;
    //{-61.5, 35.5};

    rotateToRelativeAngle(90);

/*
    std::pair<double,double> initPosition = tm->getGPSPosition();


    //tm->setInertiaHeadingToGPS(); 
    tm->setInertiaHeadingToGPS();
    tm->setManualPosition(initPosition); //TEST
    

    //Set bot at rollers and spin intake reveerse to get them
    //spinIntake(false, true);
    //vex::wait(1000, vex::msec);
    
    //Drive backward and shoot 2 diskds
    rotateAndDriveToPosition({initPosition.first + 20, initPosition.second}, false, true, false);


    rotateAndDriveToPosition({initPosition.first, initPosition.second}, false, true, true);

*/

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

void AutoDrive::rotateToRelativeAngle(double angle)  //Based on ENCODERS,
{

    double numberDriveTrainRevolutions = angle * (rc->DRIVETRAINWIDTH) * M_PI / (360 * rc->WHEELCIRC);
    double revolutionsLeftWheels = -numberDriveTrainRevolutions;
    double revolutionsRightWheels = numberDriveTrainRevolutions;

    std::pair<double, double> vel = calculateDriveTrainVel({0, rc->autoRotateVelPercent});

    hw->leftWheels.spinFor(revolutionsLeftWheels, vex::rotationUnits::rev, vel.first, vex::velocityUnits::pct, false);
    hw->rightWheels.spinFor(revolutionsRightWheels, vex::rotationUnits::rev, vel.second, vex::velocityUnits::pct);
}

void AutoDrive::rotateToHeading(double heading) {
    //Corrects heading to be from 0-360 from the x axis counterclockwise
    heading  = fmod((fmod(heading,360) + 360), 360); //fmod is modulo on doubles: heading = (heading % 360 + 360) % 360

    /* This was old for Pros  
    const double DEADBAND = 1;
    int directionMultiplier = 1;

    //Determines whether to rotate left or right based on the  shortest distance
    double differenceInHeading = heading - tm->getInertiaHeading();
    if (360 - (differenceInHeading) < differenceInHeading) directionMultiplier = -1; 
    
    while (std::abs(heading - tm->getInertiaHeading()) > DEADBAND) {
        //directionMultiplier == 1: rotate at to the left at ROTATIONVEL%. directionMultiplier == -1: rotate at to the right at ROTATIONVEL%.
        moveDriveTrain({0, -rc->autoRotateVelPercent * directionMultiplier}); 
    }   
    hw->driveTrain.stop();
    */

    //turns heading from counterclockwise to clockwise bc smartDriveTrain.turnToHeading is measured clockwisee
    heading = fmod(360.0 - heading, 360.0);
    heading = (heading >= 0 ? heading : heading + 360.0);
    if (IS_USING_INERTIA_HEADING) 
    {
    hw->smartDriveTrain.turnToHeading(heading, vex::degrees, rc->autoRotateVelPercent, vex::velocityUnits::pct);
    }
    else if (IS_USING_GPS_HEADING)
    {
        const double DEADBAND = 1;
        int directionMultiplier = 1;

        //Determines whether to rotate left or right based on the  shortest distance
        double differenceInHeading = heading - tm->getGPSHeading();
        if (360 - (differenceInHeading) < differenceInHeading) directionMultiplier = -1; 
        
        while (std::abs(heading - tm->getGPSHeading()) > DEADBAND) {
            //directionMultiplier == 1: rotate at to the left at ROTATIONVEL%. directionMultiplier == -1: rotate at to the right at ROTATIONVEL%.
            moveDriveTrain({0, -rc->autoRotateVelPercent * directionMultiplier}); 
        }   
        hw->driveTrain.stop();
    }
    else if (IS_USING_ENCODER_HEADING)
    {
        //TODO
    }
}

void AutoDrive::rotateToPosition(std::pair<double,double> finalPosition, bool ISBACKROTATION)
{
    std::pair<double,double> currPos = tm->getManualPosition();
    if (IS_USING_GPS_POSITION) currPos = tm->getGPSPosition();
    double heading = tm->getHeadingBtwnPoints(currPos, finalPosition);

    if(ISBACKROTATION) heading -= 180;
    rotateToHeading(heading);

}

void AutoDrive::rotateToPosition(GameElement* gameElement) {
    std::pair<double,double> currPos = tm->getManualPosition();
    if (IS_USING_GPS_POSITION) currPos = tm->getGPSPosition();
    double heading = tm->getHeadingBtwnPoints(currPos, gameElement->GetPositionWithMinOffset());

    if(gameElement->GetAlignment()) heading -= 180;
    rotateToHeading(heading);
}

void AutoDrive::rotateAndDriveToPosition(GameElement* element) {
    std::pair<double,double> currPos = tm->getManualPosition();
    if (IS_USING_GPS_POSITION) currPos = tm->getGPSPosition();

    std::pair<double, double> position = element->GetPositionWithMinOffset();

    rotateToPosition(element); 
    double distanceToPosition = tm->getDistanceBtwnPoints(currPos, position); //inches
    
    moveDriveTrainDistance({rc->autoDriveVelPercent, 0}, distanceToPosition);

    tm->setManualPosition(position);
}

void AutoDrive::rotateAndDriveToPosition(std::pair<double,double> position, bool ISBACKTOPOSITION) {


    rotateToPosition(position, ISBACKTOPOSITION); 

    std::pair<double,double> currPos = tm->getManualPosition();
    if (IS_USING_GPS_HEADING) currPos = tm->getGPSPosition();

    hw->controller.Screen.clearScreen();
    hw->controller.Screen.setCursor(1,1);
    hw->controller.Screen.print("init x: %.1lf y: %.1lf", currPos.first, currPos.second);

    double distanceToPosition = tm->getDistanceBtwnPoints(currPos, position); //inches



    hw->controller.Screen.setCursor(2,1);
    hw->controller.Screen.print("distance: %.1lf", distanceToPosition);
    vex::wait(200, vex::msec);
    
    moveDriveTrainDistance({rc->autoDriveVelPercent, 0}, distanceToPosition); //Drive at autoDriveVelPercent% velocity

    std::pair<double,double> temp = tm->getGPSPosition();
    hw->controller.Screen.setCursor(3,1);
    hw->controller.Screen.print("init x: %.1lf y: %.1lf", temp.first, temp.second);
    vex::wait(500, vex::msec);

    tm->setManualPosition(position);
}

void AutoDrive::rotateAndShoot(GameElement* goal, double velocityPercent, int numDisksToShoot) {
    
    rotateToPosition(goal);   //Red Goal
    shootAtDesiredVelocity(velocityPercent, numDisksToShoot);
}