#pragma once
#include "Hardware.h"

class RobotConfig {
public:
    RobotConfig(Hardware* hw);
    const int WHEELCIRC = 12.57; //Drive train wheel circumference in inches
    double lowFlywheelVoltUserDrive = 8.0;
    double highFlywheelVoltUserDrive = 12.0;
    const int SNAPSHOTSIZE = 1600;

};
