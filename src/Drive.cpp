#include "Drive.h"
#include <iostream>

Drive::Drive(Hardware* hardware, RobotConfig* robotConfig, Telemetry* telemetry) {
    hw = hardware;
    rc = robotConfig;
    tm = telemetry;

    mp = new Map();

    /* 
    X and Y coords are in inches
    Heading is in degrees from the positive x axis, rotated counterclockwise
    Wheel Displacement  is in revolutions (FL = Front Left, FR = Front Right, BL = Back Left, BR = Back Front)
    Wheel Velocity (Vel) is in RPM (
    Wheel Voltage is in volts
    Wheel Power is in watts
    */
    positionLog = new Logger(hw, "positionData.txt", {"initX", "initY", "goalX", "goalY", "gpsX", "gpsY", 
                                                        "initHeading","goalHeading", "gpsHeading", "inertiaSensorHeading",
                                                        "wheelDisplacementFL", "wheelDisplacementFR","wheelDisplacementBL","wheelDisplacementBR",
                                                        "wheelVelFL", "wheelVelFR","wheelVelBL","wheelVelBR",
                                                        "wheelVolFL", "wheelVolFR","wheelVolBL","wheelVolBR",
                                                        "wheelPowerFL", "wheelPowerFR","wheelPowerBL","wheelPowerBR",
                                                        });

    tm->setCurrPosition({0,0});
}

int Drive::logData(void* param) {
    Drive* DriveLogger = static_cast<Drive*>(param);

    while(true) {
        DriveLogger->positionLog->addData({DriveLogger->tm->getCurrPosition().first, DriveLogger->tm->getCurrPosition().second, DriveLogger->positionGoal.first, DriveLogger->positionGoal.second,DriveLogger->tm->getGPSPosition().first, DriveLogger->tm->getGPSPosition().second, 
                                DriveLogger->tm->getCurrHeading(), DriveLogger->headingGoal, DriveLogger->tm->getGPSHeading(), DriveLogger->tm->getInertiaHeading(),
                                DriveLogger->hw->wheelLeftFront.position(vex::rev), DriveLogger->hw->wheelRightFront.position(vex::rev), DriveLogger->hw->wheelLeftBack.position(vex::rev),DriveLogger->hw->wheelRightBack.position(vex::rev),
                                DriveLogger->hw->wheelLeftFront.velocity(vex::rpm), DriveLogger->hw->wheelRightFront.velocity(vex::rpm), DriveLogger->hw->wheelLeftBack.velocity(vex::rpm),DriveLogger->hw->wheelRightBack.velocity(vex::rpm),
                                DriveLogger->hw->wheelLeftFront.voltage(vex::volt), DriveLogger->hw->wheelRightFront.voltage(vex::volt), DriveLogger->hw->wheelLeftBack.voltage(vex::volt),DriveLogger->hw->wheelRightBack.voltage(vex::volt),
                                DriveLogger->hw->wheelLeftFront.power(vex::powerUnits::watt), DriveLogger->hw->wheelRightFront.power(vex::powerUnits::watt), DriveLogger->hw->wheelLeftBack.power(vex::powerUnits::watt),DriveLogger->hw->wheelRightBack.power(vex::powerUnits::watt),
                            });
        
        vex::wait(1, vex::sec);
    }
    return 0;
}

std::pair<double,double> Drive::calculateDriveTrainVel(std::pair<double,double> velPercent) //{verticalVelPercent, horizontalVelPercent}
{
        double verticalVelPercent = velPercent.first / 100;
        double horizontalVelPercent = velPercent.second / 100;

        // Calculate raw left and right motor velocity
        double rawLeftVel = verticalVelPercent + horizontalVelPercent; //raw velocity of left wheels in drive train
        double rawRightVel = verticalVelPercent - horizontalVelPercent; //raw velocity of right wheels in drive train

        // Normalize the motor velocity
        double maxRawVel = std::max(std::abs(rawLeftVel), std::abs(rawRightVel));
        double normalizationFactor = maxRawVel > 1.0 ? maxRawVel : 1.0;

        double leftVelMultiplier = rawLeftVel / normalizationFactor;
        double rightVelMultiplier = rawRightVel / normalizationFactor;


        double leftVel =  100 * leftVelMultiplier; 
        double rightVel =  100 * rightVelMultiplier;

        return {leftVel, rightVel};

}


void Drive::moveDriveTrain(std::pair<double,double> velPercent)
{
    std::pair<double,double> vel = calculateDriveTrainVel(velPercent);

    hw->leftWheels.spin(vex::directionType::fwd, vel.first, vex::velocityUnits::pct);
    hw->rightWheels.spin(vex::directionType::fwd, vel.second, vex::velocityUnits::pct);
}


void Drive ::moveDriveTrainDistance(std::pair<double,double> velPercent, double distance) {
    double numberWheelRevolutions = distance / rc->WHEELCIRC;
    
    std::pair<double, double> vel = calculateDriveTrainVel(velPercent);

    hw->wheelLeftBack.spinFor(numberWheelRevolutions, vex::rotationUnits::rev, vel.first, vex::velocityUnits::pct, false);
    hw->wheelLeftFront.spinFor(numberWheelRevolutions, vex::rotationUnits::rev, vel.first, vex::velocityUnits::pct, false);
    hw->wheelRightBack.spinFor(numberWheelRevolutions, vex::rotationUnits::rev, vel.second, vex::velocityUnits::pct, false);
    hw->wheelRightFront.spinFor(numberWheelRevolutions, vex::rotationUnits::rev, vel.second, vex::velocityUnits::pct);
    
    vex::wait(50, vex::timeUnits::msec);
    while(fabs(hw->leftWheels.velocity(vex::velocityUnits::pct)) > 0 || fabs(hw->rightWheels.velocity(vex::velocityUnits::pct)) > 0); //Blocks other tasks from starting 
    
    double currHeading = tm->getCurrHeading();
    double newX = tm->getCurrPosition().first + distance * cos(currHeading * M_PI / 180.0); //need to convert degrees to radians
    double newY = tm->getCurrPosition().second + distance * sin(currHeading * M_PI / 180.0);
    tm->setCurrPosition({newX, newY});

    vex::wait(50, vex::timeUnits::msec);
    
    if (ERROR_CORRECTION_ENABLED) {
        tm->positionErrorCorrection();
        tm->headingErrorCorrection();
    }
}

void Drive :: spinIntake(bool ISSTOP, bool ISINVERT, int volts) {
    if (ISSTOP) {
        hw->intake.stop();
    }else {
        if(ISINVERT){
            hw->intake.spin(vex::directionType::rev, volts, vex::voltageUnits::volt);
        }else{
            hw->intake.spin(vex::directionType::fwd, volts, vex::voltageUnits::volt);
        }
    }
}


double Drive::outputDistanceToGoal()
{
    return tm->getDistanceBtwnPoints(tm->getGPSPosition(), mp->mapElements.at(43)->GetPosition());
}


void Drive :: spinFlywheel(double voltage) {
    if(voltage < 0){
        hw->flywheel.spin(vex::directionType::rev, -voltage, vex::voltageUnits::volt);
    }else{
        hw->flywheel.spin(vex::directionType::fwd, voltage, vex::voltageUnits::volt);
    }
}


void Drive :: flickDisk() {

    hw->launcher.spin(vex::forward,7,vex::volt);
    wait(150,vex::msec);
    hw->launcher.spin(vex::reverse,8,vex::volt);
    wait(200,vex::msec);
    hw->launcher.stop();
}


void Drive::expand()
{
    double velPercent = 100;

    hw->expansion.spin(vex::directionType::fwd, velPercent, vex::percentUnits::pct);
    vex::wait(150, vex::timeUnits::msec);
    hw->expansion.stop();
    vex::wait(170, vex::timeUnits::msec);

}

