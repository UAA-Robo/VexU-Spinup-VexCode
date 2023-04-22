#pragma once
#include "Hardware.h"

class RobotConfig {
public:
    RobotConfig(Hardware* hardware);
    
    const double WHEELCIRC = 4 * M_PI; //Drive train wheel circumference in inches
    double lowFlywheelVoltUserDrive = 8.0;
    double highFlywheelVoltUserDrive = 12.0;
    const int SNAPSHOTSIZE = 1600;
    double autoDriveVelPercent = 20;
    double autoRotateVelPercent = 20;
    const double DRIVETRAINWIDTH = 11;//distance (in inch) between left and right side of the drivetrain (measured from the center of the wheels) 
private:
    Hardware* hw;

};
