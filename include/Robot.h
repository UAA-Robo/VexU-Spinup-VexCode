#pragma once
#include "AutoDrive.h"
#include "UserDrive.h"
#include "Telemetry.h"
#include "Hardware.h"
#include "RobotConfig.h"
#include "vex.h"


/*
STANDARD UNITS:
    Encoders in Rotations
    Distance in Inches
    Heading/Angle in Degrees
*/


class Robot{
public:
    Robot();
    void printDistToGoal();
    void drive();
    void driveAuto();
private:
    Hardware* hw;
    RobotConfig* rc;
    Telemetry* tm;

    //UserDrive* userDrive;
    AutoDrive* autoDrive;
};