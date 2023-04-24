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
    else //checks encoder heading with gps
    {
        double angleToRotate = heading - tm->getCurrHeading();
        angleToRotate = fmod(angleToRotate, 360); //make sure the angle to rotate is -360 to 360

        //Determines whether to rotate left or right based on the  shortest distance
        if (360 - fabs(angleToRotate) < angleToRotate) angleToRotate = angleToRotate - 360;
        rotateToRelativeAngle(angleToRotate);
    }
    tm->setCurrHeading(heading);
    tm->headingErrorCorrection();
}

void AutoDrive::rotateToPosition(std::pair<double,double> finalPosition, bool ISBACKROTATION)
{
    //if (IS_USING_GPS_POSITION) tm->setCurrPosition(tm->getGPSPosition());
    double heading = tm->getHeadingBtwnPoints(tm->getCurrPosition(), finalPosition);

    if(ISBACKROTATION) heading -= 180;
    rotateToHeading(heading);

}

void AutoDrive::rotateToPosition(GameElement* gameElement) {
    //if (IS_USING_GPS_POSITION) tm->setCurrPosition(tm->getGPSPosition());
    double heading = tm->getHeadingBtwnPoints(tm->getCurrPosition(), gameElement->GetPositionWithMinOffset());

    if(gameElement->GetAlignment()) heading -= 180;
    rotateToHeading(heading);
}

void AutoDrive::rotateAndDriveToPosition(GameElement* element) {
    if (IS_USING_GPS_POSITION) tm->setCurrPosition(tm->getGPSPosition());

    std::pair<double, double> position = element->GetPositionWithMinOffset();

    rotateToPosition(element); 
    double distanceToPosition = tm->getDistanceBtwnPoints(tm->getCurrPosition(), position); //inches
    if (element->GetAlignment()) distanceToPosition = -distanceToPosition;    
    moveDriveTrainDistance({rc->autoDriveVelPercent, 0}, distanceToPosition);

    tm->setCurrPosition(position);
    tm->positionErrorCorrection();
}

void AutoDrive::rotateAndDriveToPosition(std::pair<double,double> position, bool ISBACKTOPOSITION) {


    rotateToPosition(position, ISBACKTOPOSITION); 

    //if (IS_USING_GPS_HEADING) tm->setCurrPosition(tm->getGPSPosition());

    double distanceToPosition = tm->getDistanceBtwnPoints(tm->getCurrPosition(), position); //inches
    if (ISBACKTOPOSITION) distanceToPosition = -distanceToPosition;
    moveDriveTrainDistance({rc->autoDriveVelPercent, 0}, distanceToPosition); //Drive at autoDriveVelPercent% velocity

    tm->setCurrPosition(position);
    tm->positionErrorCorrection();
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

    // if (rc->quadrant == 2 && rc->teamColor == vex::color::red) q2RedPathAlgo(rc->teamColor);
    // else if (rc->quadrant == 4 && rc->teamColor == vex::color::red) q4RedPathAlgo(rc->teamColor);
    // else if (rc->quadrant == 1 && rc->teamColor == vex::color::blue) q1BluePathAlgo(rc->teamColor);
    // else if (rc->quadrant == 4 && rc->teamColor == vex::color::blue) q4BluePathAlgo(rc->teamColor);
}

void AutoDrive::q2RedPathAlgo(vex::color ourColor, bool isSkills) //Should be Granny
{
    double flywheelVelPercent = 65;
    IS_USING_GPS_HEADING = false;
    IS_USING_GPS_POSITION = false;
    IS_USING_INERTIA_HEADING = false;
    IS_USING_ENCODER_POSITION = true; //requires you to use tm->setManualPosition({x,y}) before you call autoDrive functions
    IS_USING_ENCODER_HEADING = true;   //requires you to use tm->setManualHeading(heading) before you call autoDrive functions


    std::pair<double,double> initPosition = {-61.5, 38};

    tm->setCurrPosition(initPosition); 
    tm->setCurrHeading(180);
    tm->positionErrorCorrection(); //Correct error after leaving roller
    tm->headingErrorCorrection();    

    //Set bot at rollers and spin intake reveerse to get them
    rollRoller(vex::color::red);

    std::pair<double, double> secondRollerOffset = {mp->mapElements.at(44)->GetPosition().first - 2.75, mp->mapElements.at(44)->GetPosition().second - 8}; //ONLY FOR SKILLS
    //Drive backward and shoot 2 disks
    if (isSkills){ 
        rotateAndDriveToPosition({secondRollerOffset.first, tm->getCurrPosition().second}, true);
    }else{ 
        rotateAndDriveToPosition({tm->getCurrPosition().first + 5, tm->getCurrPosition().second}, true);
        rotateAndShoot(mp->mapElements.at(43), flywheelVelPercent, 2);
    }

    if (isSkills) {
        rotateAndDriveToPosition({secondRollerOffset.first,secondRollerOffset.second}, true);
        rollRoller(ourColor);
        rotateAndDriveToPosition({tm->getCurrPosition().first, tm->getCurrPosition().first - 42}, true);
    }
    //Spin intake to pick up disks
    spinIntake(); 

    //Pick up 3 disks and move a bit past last one
    //SHOOTING DISK 25 SHOULD BE AT 62.5%
    rotateAndDriveToPosition(mp->mapElements.at(25));
    hw->controller.Screen.clearScreen();
    hw->controller.Screen.setCursor(1,1);
    hw->controller.Screen.print("%d", mp->mapElements.at(25)->GetAlignment());
    moveDriveTrainDistance({rc->autoDriveVelPercent, 0}, 4);

    //Shoot 3 disks
    rotateAndShoot(mp->mapElements.at(43), flywheelVelPercent, 3);

    //SHOOTING AT POSITION 22 SHOULD BE AROUND 60.83%
    //Pick up 3 disks and move a bit past last one
    rotateAndDriveToPosition(mp->mapElements.at(22));
    moveDriveTrainDistance({rc->autoDriveVelPercent, 0}, 4);

    //Shoot 3 disks
    rotateAndShoot(mp->mapElements.at(43), flywheelVelPercent, 3);
    
    
}

void AutoDrive::q3BluePathAlgo(vex::color ourColor, bool isSkills){

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

    tm->setCurrPosition(initPosition); 
    tm->setCurrHeading(0);
    tm->positionErrorCorrection();
    tm->headingErrorCorrection();

    //Drive to x-axis in front of roller
    rotateAndDriveToPosition({mp->mapElements.at(47)->GetPosition().first-1, initPosition.second});

    //Rotate toward roller and make contact will roller wheels
    rotateAndDriveToPosition({mp->mapElements.at(47)->GetPosition().first-1, mp->mapElements.at(44)->GetPosition().second+2});

    rollRoller(ourColor);
}

void AutoDrive::q2BluePathAlgo(vex::color ourColor, bool isSkills) //Should be Sid
{
    IS_USING_GPS_HEADING = false;
    IS_USING_GPS_POSITION = false;
    IS_USING_INERTIA_HEADING = false;
    IS_USING_ENCODER_POSITION = true; //requires you to use tm->setManualPosition({x,y}) before you call autoDrive functions
    IS_USING_ENCODER_HEADING = true;   //requires you to use tm->setManualHeading(heading) before you call autoDrive functions

    std::pair<double,double> initPosition = {-16, 56};

    tm->setCurrPosition(initPosition);
    tm->setCurrHeading(180);
    tm->positionErrorCorrection();
    tm->headingErrorCorrection();
    
    shootAtDesiredVelocity(40, 2);

    spinFlywheel(0);

    rotateAndDriveToPosition({mp->mapElements.at(44)->GetPosition().first+5, initPosition.second});
    rotateAndDriveToPosition({mp->mapElements.at(44)->GetPosition().first+5, mp->mapElements.at(47)->GetPosition().second-2});

    rollRoller(rc->teamColor);


}

void AutoDrive::q4BluePathAlgo(vex::color ourColor, bool isSkills) //Should be Granny
{
    
}

void AutoDrive::rollRoller(vex::color ourColor)
{
    const double HUE_DEADBAND = 40;
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