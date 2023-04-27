#pragma once
#include "Hardware.h"

class RobotConfig {
public:
    RobotConfig(Hardware* hw);
    void setTeamColor(std::pair<double, double> locale);
    void setQuadrant(std::pair<double, double> locale);
    const double WHEELCIRC = 4 * M_PI; //Drive train wheel circumference in inches
    double lowFlywheelVoltUserDrive = 6.0;
    double highFlywheelVoltUserDrive = 8.0;
    const int SNAPSHOTSIZE = 1600;
    double autoDriveVelPercent = 45;
    double autoRotateVelPercent = 20;
    const double DRIVETRAINWIDTH = 11;//distance (in inch) between left and right side of the drivetrain (measured from the center of the wheels) 
    vex::color teamColor;
    int quadrant;
    double PID_INTERVAL = 1;
    private:
    Hardware* hw;

};
