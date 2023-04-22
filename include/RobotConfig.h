#pragma once
#include "Hardware.h"

class RobotConfig {
public:
    RobotConfig(Hardware* hw);
    void setTeamColor(std::pair<double, double> locale);
    void setQuadrant(std::pair<double, double> locale);
    const double WHEELCIRC = 4 * M_PI; //Drive train wheel circumference in inches
    double lowFlywheelVoltUserDrive = 8.0;
    double highFlywheelVoltUserDrive = 12.0;
    const int SNAPSHOTSIZE = 1600;
    double autoDriveVelPercent = 20;
    double autoRotateVelPercent = 20;
    vex::color teamColor;
    int quadrant;
    private:
    Hardware* hw;

};
