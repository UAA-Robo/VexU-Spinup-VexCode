#include "AutoDrive.h"

AutoDrive::AutoDrive(Hardware* hardware, RobotConfig* robotConfig, Telemetry* telemetry): Drive(hardware, robotConfig, telemetry) {
    
}
void AutoDrive::drive() {
    usePathing();
}

void AutoDrive::shootAtDesiredVelocity(double velocityPercent, int numFlicks)
{   
    double desiredVoltage = velocityPercent / 100.0 * 12.0;
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

    rotateToPosition(gameElement->GetPosition(), gameElement->GetAlignment());

    // std::pair<double,double> currPos = tm->getManualPosition();
    // if (IS_USING_GPS_POSITION) currPos = tm->getGPSPosition();
    // double heading = tm->getHeadingBtwnPoints(currPos, gameElement->GetPositionWithMinOffset());

    // if(gameElement->GetAlignment()) heading -= 180;
    // rotateToHeading(heading);
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
    rc->setTeamColor(tm->getGPSPosition());

   // hw->controller.Screen.print("path algo reached");

   if(rc->teamColor == vex::color::red){
        switch(rc->quadrant){
            case 1:
            break;

            case 2:
            q2RedPathAlgo(rc->teamColor, false);
            break;

            case 3:
            break;

            case 4:
            q4RedPathAlgo(rc->teamColor, false);
            break;
        }
   }else{
        switch(rc->quadrant){
            case 1:

            break;

            case 2:
            q2BluePathAlgo(rc->teamColor, false);
            break;

            case 3:  
            break;

            case 4:
            q4BluePathAlgo(rc->teamColor, false);
            break;
        }
   }
}

void AutoDrive::q2RedPathAlgo(vex::color ourColor, bool isSkills) //Should be Granny
{
    double flywheelVelPercent = 66;
    IS_USING_GPS_HEADING = false;
    IS_USING_GPS_POSITION = false;
    IS_USING_INERTIA_HEADING = false;
    IS_USING_ENCODER_POSITION = true; //requires you to use tm->setManualPosition({x,y}) before you call autoDrive functions
    IS_USING_ENCODER_HEADING = true;   //requires you to use tm->setManualHeading(heading) before you call autoDrive functions

    std::pair<double,double> initPosition = {-61.5, 38};

    tm->setManualPosition(initPosition); 
    tm->setManualHeading(180);
    
    //Set bot at rollers and spin intake reveerse to get them
    rollRoller(ourColor); //red

    //Drive backward and shoot 2 diskds
    rotateAndDriveToPosition({initPosition.first + 5, initPosition.second}, true);
    rotateAndShoot(mp->mapElements.at(43), flywheelVelPercent, 2);

    
    //Spin intake to pick up disks
    spinIntake(); 

    //Pick up 3 disks and move a bit past last one, and shoot them
    rotateAndDriveToPosition(mp->mapElements.at(25));
    moveDriveTrainDistance({rc->autoDriveVelPercent, 0}, 5); //Move 5 extra inches past disk
    rotateAndShoot(mp->mapElements.at(43), flywheelVelPercent - 5, 3);
    

    //Pick up 3 disks and move a bit past last one, and shoot them
    /*
    rotateAndDriveToPosition(mp->mapElements.at(22));
    moveDriveTrainDistance({rc->autoDriveVelPercent, 0}, 3);
    rotateAndShoot(mp->mapElements.at(43), flywheelVelPercent, 3);
    */
    rotateAndDriveToPosition(mp->mapElements.at(24));
    rotateAndDriveToPosition(mp->mapElements.at(23));
    rotateAndDriveToPosition(mp->mapElements.at(22));
    moveDriveTrainDistance({rc->autoDriveVelPercent, 0}, 3);
    
    rotateAndShoot(mp->mapElements.at(43), flywheelVelPercent, 3);

    if(!isSkills) return;
    
}

void AutoDrive::q4BluePathAlgo(vex::color ourColor, bool isSkills){
    double flywheelVelPercent = 66;
    IS_USING_GPS_HEADING = false;
    IS_USING_GPS_POSITION = false;
    IS_USING_INERTIA_HEADING = false;
    IS_USING_ENCODER_POSITION = true; //requires you to use tm->setManualPosition({x,y}) before you call autoDrive functions
    IS_USING_ENCODER_HEADING = true;   //requires you to use tm->setManualHeading(heading) before you call autoDrive functions

    std::pair<double,double> initPosition = {61.5, -38};

    tm->setManualPosition(initPosition); 
    tm->setManualHeading(0);
    
    //Set bot at rollers and spin intake reveerse to get them
    rollRoller(ourColor); //blue

    //Drive backward and shoot 2 diskds
    rotateAndDriveToPosition({initPosition.first - 5, initPosition.second}, true);
    rotateAndShoot(mp->mapElements.at(42), flywheelVelPercent, 2);

    
    //Spin intake to pick up disks
    spinIntake(); 

    //Pick up 3 disks and move a bit past last one, and shoot them
    rotateAndDriveToPosition(mp->mapElements.at(32));
    moveDriveTrainDistance({rc->autoDriveVelPercent, 0}, 5); //Move 5 extra inches past disk
    rotateAndShoot(mp->mapElements.at(42), flywheelVelPercent - 5, 3);
    

    //Pick up 3 disks and move a bit past last one, and shoot them
    /*
    rotateAndDriveToPosition(mp->mapElements.at(22));
    moveDriveTrainDistance({rc->autoDriveVelPercent, 0}, 3);
    rotateAndShoot(mp->mapElements.at(43), flywheelVelPercent, 3);
    */
    rotateAndDriveToPosition(mp->mapElements.at(33));
    rotateAndDriveToPosition(mp->mapElements.at(34));
    rotateAndDriveToPosition(mp->mapElements.at(35));
    moveDriveTrainDistance({rc->autoDriveVelPercent, 0}, 3);
    
    rotateAndShoot(mp->mapElements.at(42), flywheelVelPercent, 3);

    /*
    PLACE NORMAL COMPETITION PATHING BEFORE IF STATEMENT HERE
    */

    if(!isSkills) return;

    /*
    CONTINUE EXTENDED SKILLS PATHING HERE
    */
}

void AutoDrive::q4RedPathAlgo(vex::color ourColor, bool isSkills) //Should be Sid
{
    IS_USING_GPS_HEADING = false;
    IS_USING_GPS_POSITION = false;
    IS_USING_INERTIA_HEADING = false;
    IS_USING_ENCODER_POSITION = true; //requires you to use tm->setManualPosition({x,y}) before you call autoDrive functions
    IS_USING_ENCODER_HEADING = true;   //requires you to use tm->setManualHeading(heading) before you call autoDrive functions

    std::pair<double,double> initPosition = {15, -59};

    shootAtDesiredVelocity(40, 2);

    spinFlywheel(0);

    tm->setManualPosition(initPosition); 
    tm->setManualHeading(0);

    //Drive to x-axis in front of roller
    rotateAndDriveToPosition({mp->mapElements.at(44)->GetPosition().first-1, initPosition.second});

    //Rotate toward roller and make contact will roller wheels
    rotateAndDriveToPosition({mp->mapElements.at(44)->GetPosition().first-1, mp->mapElements.at(44)->GetPosition().second+2});

    rollRoller(ourColor);

    if(!isSkills) return;
}

void AutoDrive::q2BluePathAlgo(vex::color ourColor, bool isSkills) //Should be Sid
{
    IS_USING_GPS_HEADING = false;
    IS_USING_GPS_POSITION = false;
    IS_USING_INERTIA_HEADING = false;
    IS_USING_ENCODER_POSITION = true; //requires you to use tm->setManualPosition({x,y}) before you call autoDrive functions
    IS_USING_ENCODER_HEADING = true;   //requires you to use tm->setManualHeading(heading) before you call autoDrive functions

    std::pair<double,double> initPosition = {-16, 56};

    tm->setManualPosition(initPosition);
    tm->setManualHeading(180);
    
    shootAtDesiredVelocity(40, 2);

    spinFlywheel(0);

    rotateAndDriveToPosition({mp->mapElements.at(47)->GetPosition().first+5, initPosition.second});
    rotateAndDriveToPosition({mp->mapElements.at(47)->GetPosition().first+5, mp->mapElements.at(47)->GetPosition().second-2});

    rollRoller(rc->teamColor);

    if(!isSkills) return;

}

void AutoDrive::q4BluePathAlgo(vex::color ourColor, bool isSkills) //Should be Granny
{
    
}

void AutoDrive::rollRoller(vex::color ourColor)
{
    const double HUE_DEADBAND = 10;
    const double intakeVolt = 9; //reaches 150 rpm to match the optical sensor rate
    double oppositeHue;

    if (ourColor == vex::color::red) oppositeHue = 220;   //red hue = 5
    else if (ourColor == vex::color::blue) oppositeHue = 5; //blue hue = 220

    spinIntake(false, true, 9);
    while(fabs(hw->opticalSensor.hue() - oppositeHue) < HUE_DEADBAND); //Spin while seeing opposite color (outside deadband)
    spinIntake(true, true);
}

void AutoDrive::centerOnDisk(){
    int isRight = 1;
    this->hw->visionSensor.takeSnapshot(1);
    // 100 is a safe value for size of disk. Tested in poor lighting, please fine tune in better lighting.
    if(this->hw->visionSensor.largestObject.width > 100){
        // diff is the offset from the middle of the screen. Correct to 320 once sensor is centered over intake.
        // 75 / 640 is Camera FOV / Camera Resolution. This should allow for a pix to deg conversion.
        int diff = (75.f / 640.f) * (this->hw->visionSensor.largestObject.centerX - 200);
        // Minimum turn dist
        int minTurn = 5;
        // Correct for direction
        if(diff < 0) isRight = -1;
        // Return if Disk is close enough to the center of the screen
        if(diff < 2){
            return;
        }else if(std::fabs(diff) < minTurn){
            // Negative value is to prevent the robot from always rotating away from the disk
            // We overshoot the rotation, because the robot can't turn small angles accurately, but can with large angles
            // So we overshoot so we can correct the angle with a big turn.
            rotateToRelativeAngle(-(diff + minTurn * isRight));
            rotateToRelativeAngle(minTurn * isRight);
        }else{
            rotateToRelativeAngle(-diff);
        }
    }
}
