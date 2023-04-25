#include "AutoDrive.h"

AutoDrive::AutoDrive(Hardware *hardware, RobotConfig *robotConfig, Telemetry *telemetry) : Drive(hardware, robotConfig, telemetry)
{
}

void AutoDrive::drive()
{   
    //shootAtDesiredVelocity(75, 10);
    isSkills = true;
    usePathing();
}

void AutoDrive::shootAtDesiredVelocity(double velocityPercent, int numFlicks)
{
    double desiredVoltage = velocityPercent / 100.0 * 12.0;
    spinFlywheel(desiredVoltage);
    vex::wait(500, vex::msec);
    for (int i = 0; i < numFlicks; ++i)
    {
        //vex::wait(1000, vex::msec);
        while (hw->flywheel.velocity(vex::percent) < velocityPercent) {
            hw->controller.Screen.clearScreen();
            hw->controller.Screen.setCursor(1,1);
            hw->controller.Screen.print("%.2lf", hw->flywheel.velocity(vex::percent));
            //vex::wait(200, vex::msec);
        }
        vex::wait(1500, vex::msec);
        flickDisk();
        vex::wait(500, vex::msec);
    }
}

double AutoDrive::getPidFlywheelVoltage(double targetVoltage)
{
    double maxRPM = 600;
    double targetRPM = targetVoltage / 12.0 * maxRPM;
    double currentRPM = (hw->flywheelTop.velocity(vex::rpm) + hw->flywheelBottom.velocity(vex::rpm)) / 2;// = (flywheelTopMotor.velocity(rpm) + flywheelBottomMotor.velocity(rpm)) / 2;


    this->Kp = 0.9/8.0 * targetVoltage;
    this->Kd = 0.6 / 8.0 * targetVoltage;
    this->Ki = 0.3 / 8.0 * targetVoltage;
    // PID calculations
    this->error = targetRPM - currentRPM;
    this->error = this->error / maxRPM * 12;
    this->integral += error;
    this->derivative = error - prevError;
    this->output = Kp * error + Ki * integral + Kd * derivative;
    this->prevError = error;
    //this->outPutVolt = output / maxRPM * 12;

    // Limit output to valid motor voltage
    if (output > 12.0)
        output = 12.0;
    if (output < -12.0)
        output = -12.0;

    return output;
}

void AutoDrive::rotateToRelativeAngle(double angle) // Based on ENCODERS,
{

    double numberDriveTrainRevolutions = angle * (rc->DRIVETRAINWIDTH) * M_PI / (360 * rc->WHEELCIRC);
    double revolutionsLeftWheels = -numberDriveTrainRevolutions;
    double revolutionsRightWheels = numberDriveTrainRevolutions;

    std::pair<double, double> vel = calculateDriveTrainVel({0, rc->autoRotateVelPercent});

    hw->leftWheels.spinFor(revolutionsLeftWheels, vex::rotationUnits::rev, vel.first, vex::velocityUnits::pct, false);
    hw->rightWheels.spinFor(revolutionsRightWheels, vex::rotationUnits::rev, vel.second, vex::velocityUnits::pct);
    while (fabs(hw->leftWheels.velocity(vex::velocityUnits::pct)) > 0 || fabs(hw->rightWheels.velocity(vex::velocityUnits::pct)) > 0); // Blocks other tasks from starting
}

void AutoDrive::rotateToHeading(double heading)
{
    // Corrects heading to be from 0-360 from the x axis counterclockwise if applicable
    heading = fmod(heading, 360);
    if (heading < 0)
        heading += 360;

    if (IS_USING_INERTIA_HEADING)
    {
        // turns heading from counterclockwise to clockwise bc smartDriveTrain.turnToHeading is measured clockwisee
        int clockwiseHeading = fmod(360.0 - heading, 360.0);
        clockwiseHeading = (heading >= 0 ? heading : heading + 360.0);

        hw->smartDriveTrain.turnToHeading(clockwiseHeading, vex::degrees, rc->autoRotateVelPercent, vex::velocityUnits::pct);
    }
    else // checks encoder heading with gps
    {
        double angleToRotate = heading - tm->getCurrHeading();
        angleToRotate = fmod(angleToRotate, 360); // make sure the angle to rotate is -360 to 360

        // Determines whether to rotate left or right based on the  shortest distance
        if (360 - fabs(angleToRotate) < angleToRotate)
            angleToRotate = angleToRotate - 360;
        rotateToRelativeAngle(angleToRotate + robotAngleOffset);
    }
    tm->setCurrHeading(heading);
    tm->headingErrorCorrection();
}

void AutoDrive::rotateToPosition(std::pair<double, double> finalPosition, bool ISBACKROTATION)
{
    // if (IS_USING_GPS_POSITION) tm->setCurrPosition(tm->getGPSPosition());
    double heading = tm->getHeadingBtwnPoints(tm->getCurrPosition(), finalPosition);

    if (ISBACKROTATION)
        heading -= 180;
    rotateToHeading(heading);
}

void AutoDrive::rotateToPosition(GameElement *gameElement)
{
    // if (IS_USING_GPS_POSITION) tm->setCurrPosition(tm->getGPSPosition());
    double heading = tm->getHeadingBtwnPoints(tm->getCurrPosition(), gameElement->GetPositionWithMinOffset());

    if (gameElement->GetAlignment())
        heading -= 180;
    rotateToHeading(heading);
}

void AutoDrive::rotateAndDriveToPosition(GameElement *element)
{
    // if (IS_USING_GPS_POSITION) tm->setCurrPosition(tm->getGPSPosition());

    std::pair<double, double> position = element->GetPositionWithMinOffset();

    rotateToPosition(element);
    double distanceToPosition = tm->getDistanceBtwnPoints(tm->getCurrPosition(), position); // inches
    if (element->GetAlignment())
        distanceToPosition = -distanceToPosition;
    moveDriveTrainDistance({rc->autoDriveVelPercent, 0}, distanceToPosition);
}

void AutoDrive::rotateAndDriveToPosition(std::pair<double, double> position, bool ISBACKTOPOSITION)
{

    rotateToPosition(position, ISBACKTOPOSITION);

    // if (IS_USING_GPS_HEADING) tm->setCurrPosition(tm->getGPSPosition());

    double distanceToPosition = tm->getDistanceBtwnPoints(tm->getCurrPosition(), position); // inches
    if (ISBACKTOPOSITION)
        distanceToPosition = -distanceToPosition;
    moveDriveTrainDistance({rc->autoDriveVelPercent, 0}, distanceToPosition); // Drive at autoDriveVelPercent% velocity
}

void AutoDrive::rotateAndShoot(std::pair<double, double> goal, double velocityPercent, int numDisksToShoot)
{

    rotateToPosition(goal, true);
    shootAtDesiredVelocity(velocityPercent, numDisksToShoot);
}
void AutoDrive::rotateAndShoot(GameElement *goal, double velocityPercent, int numDisksToShoot)
{
    rotateToPosition(goal);
    shootAtDesiredVelocity(velocityPercent, numDisksToShoot);
}

void AutoDrive::usePathing()
{
    rc->setTeamColor(tm->getGPSPosition());

    // hw->controller.Screen.print("path algo reached");

    if (rc->teamColor == vex::color::red)
    {
        switch (rc->quadrant)
        {
        case 1:
            break;

        case 2:
            q2RedPathAlgo(rc->teamColor);
            break;

        case 3:
            break;

        case 4:
            q4RedPathAlgo(rc->teamColor);
            break;
        }
    }
    else
    {
        switch (rc->quadrant)
        {
        case 1:

            break;

        case 2:
            q2BluePathAlgo(rc->teamColor);
            break;

        case 3:
            break;

        case 4:
            q4BluePathAlgo(rc->teamColor);
            break;
        }
    }

    // if (rc->quadrant == 2 && rc->teamColor == vex::color::red) q2RedPathAlgo(rc->teamColor);
    // else if (rc->quadrant == 4 && rc->teamColor == vex::color::red) q4RedPathAlgo(rc->teamColor);
    // else if (rc->quadrant == 1 && rc->teamColor == vex::color::blue) q1BluePathAlgo(rc->teamColor);
    // else if (rc->quadrant == 4 && rc->teamColor == vex::color::blue) q4BluePathAlgo(rc->teamColor);
}

void AutoDrive::q2RedPathAlgo(vex::color ourColor) // Should be Granny
{
    // double flywheelVelPercent = 65;
    IS_USING_GPS_HEADING = false;
    IS_USING_GPS_POSITION = false;
    IS_USING_INERTIA_HEADING = false;
    IS_USING_ENCODER_POSITION = true; // requires you to use tm->setManualPosition({x,y}) before you call autoDrive functions
    IS_USING_ENCODER_HEADING = true;  // requires you to use tm->setManualHeading(heading) before you call autoDrive functions

    // robotAngleOffset = 0.5; //degrees
    std::pair<double, double> initPosition = {-61.5, 38};

    tm->setCurrPosition(initPosition);
    tm->setCurrHeading(180);
    //tm->positionErrorCorrection();
    //tm->headingErrorCorrection();

    // Set bot at rollers and spin intake reveerse to get them
    rollRoller(vex::color::red);

    std::pair<double, double> secondRollerOffset = {mp->mapElements.at(44)->GetPosition().first - 2, mp->mapElements.at(44)->GetPosition().second - 9}; // ONLY FOR SKILLS
    // Drive backward and shoot 2 disks
    if (isSkills)
    {
        rotateAndDriveToPosition({secondRollerOffset.first, tm->getCurrPosition().second}, true);
    }
    else
    {
        rotateAndDriveToPosition({tm->getCurrPosition().first + 5, tm->getCurrPosition().second}, true);
        
    }
    rotateAndShoot(mp->mapElements.at(43), 65, 2);

    if (isSkills)
    {
        rotateAndDriveToPosition({secondRollerOffset.first, secondRollerOffset.second}, false);
        rollRoller(vex::color::red);
        rotateAndDriveToPosition({tm->getCurrPosition().first, tm->getCurrPosition().second - 38}, true);
    }
    // Spin intake to pick up disks
    spinIntake();

    // Pick up 3 disks and move a bit past last one and shoot them
    // SHOOTING DISK 25 SHOULD BE AT 62.5%
    rotateAndDriveToPosition(mp->mapElements.at(25));
    moveDriveTrainDistance({rc->autoDriveVelPercent, 0}, 15);
    rotateAndShoot(mp->mapElements.at(43), 65, 3);

    // SHOOTING AT POSITION 22 SHOULD BE AROUND 60.83%
    // Pick up 3 disks and move a bit past last one
    rotateAndDriveToPosition(mp->mapElements.at(24));
    rotateAndDriveToPosition(mp->mapElements.at(23));
    rotateAndDriveToPosition(mp->mapElements.at(22));
    moveDriveTrainDistance({rc->autoDriveVelPercent, 0}, 0);

    // Shoot 3 disks
    rotateAndShoot(mp->mapElements.at(43), 62, 3);

    if (isSkills) { //expand
        rotateAndDriveToPosition({58,-58}, true);
        rotateToPosition({10, 70});
        for (int i = 0; i < 3; i++) expand();
    }
}

void AutoDrive::q3BluePathAlgo(vex::color ourColor)
{
}

void AutoDrive::q4RedPathAlgo(vex::color ourColor) // Should be Sid
{
    IS_USING_GPS_HEADING = false;
    IS_USING_GPS_POSITION = false;
    IS_USING_INERTIA_HEADING = false;
    IS_USING_ENCODER_POSITION = true; // requires you to use tm->setManualPosition({x,y}) before you call autoDrive functions
    IS_USING_ENCODER_HEADING = true;  // requires you to use tm->setManualHeading(heading) before you call autoDrive functions

    std::pair<double, double> initPosition = {16.5, -56};
    tm->setCurrPosition(initPosition);
    tm->setCurrHeading(0);

    shootAtDesiredVelocity(40, 2);
    spinFlywheel(0);

    tm->setCurrPosition(initPosition);
    tm->setCurrHeading(0);
    tm->positionErrorCorrection();
    // tm->headingErrorCorrection();

    double xRollerOffset = 2;
    double yRollerOffst = 6.5;
    // Drive to x-axis in front of roller
    rotateAndDriveToPosition({mp->mapElements.at(47)->GetPosition().first - xRollerOffset, initPosition.second});
    // Rotate toward roller and make contact will roller wheels
    rotateAndDriveToPosition({mp->mapElements.at(47)->GetPosition().first - xRollerOffset, mp->mapElements.at(47)->GetPosition().second + yRollerOffst});

    rollRoller(ourColor, false);
}

void AutoDrive::q2BluePathAlgo(vex::color ourColor) // Should be Sid
{
    IS_USING_GPS_HEADING = false;
    IS_USING_GPS_POSITION = false;
    IS_USING_INERTIA_HEADING = false;
    IS_USING_ENCODER_POSITION = true; // requires you to use tm->setManualPosition({x,y}) before you call autoDrive functions
    IS_USING_ENCODER_HEADING = true;  // requires you to use tm->setManualHeading(heading) before you call autoDrive functions

    std::pair<double, double> initPosition = {-16, 56};
    tm->setCurrPosition(initPosition);
    tm->setCurrHeading(180);

    shootAtDesiredVelocity(40, 2);
    spinFlywheel(0);

    tm->setCurrPosition(initPosition);

    double xRollerOffset = 2;
    double yRollerOffst = -6.5;
    // Drive to x-axis in front of roller
    rotateAndDriveToPosition({mp->mapElements.at(44)->GetPosition().first + xRollerOffset, initPosition.second});
        // Rotate toward roller and make contact will roller wheels
    rotateAndDriveToPosition({mp->mapElements.at(44)->GetPosition().first + xRollerOffset, mp->mapElements.at(44)->GetPosition().second + yRollerOffst});

    rollRoller(rc->teamColor);
}

void AutoDrive::q4BluePathAlgo(vex::color ourColor) // Should be Granny
{
    IS_USING_GPS_HEADING = false;
    IS_USING_GPS_POSITION = false;
    IS_USING_INERTIA_HEADING = false;
    IS_USING_ENCODER_POSITION = true; // requires you to use tm->setManualPosition({x,y}) before you call autoDrive functions
    IS_USING_ENCODER_HEADING = true;  // requires you to use tm->setManualHeading(heading) before you call autoDrive functions

    std::pair<double, double> initPosition = {61.5, -38};

    tm->setCurrPosition(initPosition);
    tm->setCurrHeading(0);
    // Set bot at rollers and spin intake reveerse to get the
    rollRoller(ourColor); // blue

    // Drive backward and shoot 2 diskds
    rotateAndDriveToPosition({initPosition.first - 5, initPosition.second}, true);
    rotateAndShoot(mp->mapElements.at(42), 68, 2);

    // Spin intake to pick up disks
    spinIntake();

    // Pick up 3 disks and move a bit past last one, and shoot them
    rotateAndDriveToPosition(mp->mapElements.at(32));
    moveDriveTrainDistance({rc->autoDriveVelPercent, 0}, 15); // Move 5 extra inches past disk
    rotateAndShoot(mp->mapElements.at(42), 62, 3);

    // Pick up 3 disks and move a bit past last one, and shoot them
    /*
    rotateAndDriveToPosition(mp->mapElements.at(22));
    moveDriveTrainDistance({rc->autoDriveVelPercent, 0}, 3);
    rotateAndShoot(mp->mapElements.at(43), flywheelVelPercent, 3);
    */
    rotateAndDriveToPosition(mp->mapElements.at(33));
    rotateAndDriveToPosition(mp->mapElements.at(34));
    rotateAndDriveToPosition(mp->mapElements.at(35));
    moveDriveTrainDistance({rc->autoDriveVelPercent, 0}, 3);

    rotateAndShoot(mp->mapElements.at(42), 65, 3);
}

void AutoDrive::rollRoller(vex::color ourColor, bool IS_NO_TIME_OUT)
{
    const double HUE_DEADBAND = 40;
    const double INTAKE_VOLT = 9; // reaches 150 rpm to match the optical sensor rate
    double oppositeHue;
    double ourHue;
    const double RED_HUE = 5;
    const double BLUE_HUE = 220;
    double initTime = hw->brain.timer(vex::timeUnits::sec);
    const double MAX_TIME = 6; // seconds

    hw->controller.Screen.print(hw->opticalSensor.hue());
    if (ourColor == vex::color::red)
        oppositeHue = BLUE_HUE;
    else
        oppositeHue = RED_HUE;

    spinIntake(false, true, 9);

    // If not red or blue, spin for
    //  if (fabs(hw->opticalSensor.hue() - RED_HUE) > HUE_DEADBAND && fabs(hw->opticalSensor.hue() - BLUE_HUE) > HUE_DEADBAND) {
    //      vex::wait(2,vex::timeUnits::sec);
    //  }
    // Else spin while seeing opposite color (outside deadband) and for 5 secs max

    while ((fabs(hw->opticalSensor.hue() - oppositeHue) < HUE_DEADBAND || hw->opticalSensor.hue() >= 360 - (HUE_DEADBAND - fabs(hw->opticalSensor.hue() - oppositeHue))) && (((hw->brain.timer(vex::timeUnits::sec) - initTime) < MAX_TIME) || IS_NO_TIME_OUT))
        ;

    spinIntake(true, true); // stop intake
}

void AutoDrive::centerOnDisk()
{
    int isRight = 1;
    this->hw->visionSensor.takeSnapshot(1);
    // 100 is a safe value for size of disk. Tested in poor lighting, please fine tune in better lighting.
    if (this->hw->visionSensor.largestObject.width > 100)
    {
        // diff is the offset from the middle of the screen. Correct to 320 once sensor is centered over intake.
        // 75 / 640 is Camera FOV / Camera Resolution. This should allow for a pix to deg conversion.
        int diff = (75.f / 640.f) * (this->hw->visionSensor.largestObject.centerX - 200);
        // Minimum turn dist
        int minTurn = 5;
        // Correct for direction
        if (diff < 0)
            isRight = -1;
        // Return if Disk is close enough to the center of the screen
        if (diff < 2)
        {
            return;
        }
        else if (std::fabs(diff) < minTurn)
        {
            // Negative value is to prevent the robot from always rotating away from the disk
            // We overshoot the rotation, because the robot can't turn small angles accurately, but can with large angles
            // So we overshoot so we can correct the angle with a big turn.
            rotateToRelativeAngle(-(diff + minTurn * isRight));
            rotateToRelativeAngle(minTurn * isRight);
        }
        else
        {
            rotateToRelativeAngle(-diff);
        }
    }
}