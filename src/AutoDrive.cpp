#include "AutoDrive.h"

AutoDrive::AutoDrive(Hardware* hardware, RobotConfig* robotConfig, Telemetry* telemetry): Drive(hardware, robotConfig, telemetry) {
    
}

void AutoDrive::drive() {
    IS_USING_GPS_HEADING = false;
    IS_USING_GPS_POSITION = false;
    IS_USING_INERTIA_HEADING = false;
    IS_USING_ENCODER_POSITION = true; //requires you to use tm->setManualPosition({x,y}) before you call autoDrive functions
    IS_USING_ENCODER_HEADING = true;   //requires you to use tm->setManualHeading(heading) before you call autoDrive functions

    /* //TEST
    std::pair<double,double> initPosition = {0, 0};
    tm->setManualHeading(90);
    tm->setManualPosition(initPosition);

    rotateAndDriveToPosition({-5, 0}, true);
    */


    //tm->setInertiaHeadingToGPS(); 
    tm->setInertiaHeadingToGPS();
    tm->setManualPosition(initPosition); //TEST
    

    //Set bot at rollers and spin intake reveerse to get them
    spinIntake(false, true);
    vex::wait(1000, vex::msec);
    
    //Drive backward and shoot 2 diskds
    rotateAndDriveToPosition({initPosition.first + 20, initPosition.second}, false, true, false);


    rotateAndDriveToPosition({initPosition.first, initPosition.second}, false, true, true);

    
    
    rotateAndShoot(mp->mapElements.at(43), rc->lowFlywheelVoltUserDrive, 2);
    /*
    
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
    //Corrects heading to be from 0-360 from the x axis counterclockwise if applicable
    heading = fmod(heading,360);
    if (heading < 0) heading += 360;
   

    if (IS_USING_INERTIA_HEADING) 
    {
        //turns heading from counterclockwise to clockwise bc smartDriveTrain.turnToHeading is measured clockwisee
        int clockwiseHeading = fmod(360.0 - heading, 360.0);
        clockwiseHeading = (heading >= 0 ? heading : heading + 360.0);
    
        hw->smartDriveTrain.turnToHeading(clockwiseHeading, vex::degrees, rc->autoRotateVelPercent, vex::velocityUnits::pct);
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
        double currHeading = tm->getManualHeading();
        double angleToRotate = heading - currHeading;
        angleToRotate = fmod(angleToRotate, 360); //make sure the angle to rotate is -360 to 360

        //Determines whether to rotate left or right based on the  shortest distance
        if (360 - fabs(angleToRotate) < angleToRotate) angleToRotate = angleToRotate - 360;
        rotateToRelativeAngle(angleToRotate);
    }
    tm->setManualHeading(heading);
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
    if (element->GetAlignment()) distanceToPosition = -distanceToPosition;    
    moveDriveTrainDistance({rc->autoDriveVelPercent, 0}, distanceToPosition);

    tm->setManualPosition(position);
}

void AutoDrive::rotateAndDriveToPosition(std::pair<double,double> position, bool ISBACKTOPOSITION) {


    rotateToPosition(position, ISBACKTOPOSITION); 

    std::pair<double,double> currPos = tm->getManualPosition();
    if (IS_USING_GPS_HEADING) currPos = tm->getGPSPosition();

    double distanceToPosition = tm->getDistanceBtwnPoints(currPos, position); //inches
    if (ISBACKTOPOSITION) distanceToPosition = -distanceToPosition;
    moveDriveTrainDistance({rc->autoDriveVelPercent, 0}, distanceToPosition); //Drive at autoDriveVelPercent% velocity

    tm->setManualPosition(position);
}

void AutoDrive::rotateAndShoot(GameElement* goal, double velocityPercent, int numDisksToShoot) {
    
    rotateToPosition(goal);   //Red Goal
    shootAtDesiredVelocity(velocityPercent, numDisksToShoot);
}

void AutoDrive::usePathing(){
    rc->setTeamColor({1, -1});

   // hw->controller.Screen.print("path algo reached");

    switch(rc->quadrant){
        case 1:
           // hw->controller.Screen.print("q1 code reached");
            q1PathAlgo(rc->teamColor);
        break;
        case 2:
            //q2PathAlgo(rc->teamColor);
        break;
        case 3:
           // q3PathAlgo(rc->teamColor);
        break;
        case 4:
            //hw->controller.Screen.print("path algo reached");
           q4PathAlgo(rc->teamColor);
        break;
    }
}

void AutoDrive::q1PathAlgo(vex::color ourColor)
{
    rollRoller(ourColor);
}

void AutoDrive::q4PathAlgo(vex::color ourColor)
{
    rollRoller(ourColor);
}

void AutoDrive::rollRoller(vex::color ourColor)
{
    hw->opticalSensor.setLight(vex::ledState::on);
   while(true){
        hw->controller.Screen.clearScreen();
        hw->controller.Screen.setCursor(1,1);
        hw->controller.Screen.print("%.2f", hw->opticalSensor.hue());
        vex::wait(500, vex::timeUnits::msec);
    }

    
  //  while(hw->opticalSensor.color() != ourColor){
  //      spinIntake(false, false, 8);
  //  }
   // spinIntake(true, false);
}
