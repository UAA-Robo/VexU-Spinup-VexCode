#include "Drive.h"


std::pair<double,double> Drive::calculateDriveTrainVel(std::pair<double,double> velPercent) //{verticalVelPercent, horizontalVelPercent}
{
        double verticalVelPercent = velPercent.first / 100;
        double horizontalVelPercent = velPercent.second / 100;

        // Calculate raw left and right motor velocity
        double rawLeftVel = verticalVelPercent + horizontalVelPercent; //raw velocity of left wheels in drive train
        double rawRightVel = verticalVelPercent - horizontalVelPercent; //raw velocity of right wheels in drive train

        // Normalize the motor velocity
        double maxRawVel = std::max(std::abs(rawLeftVel), std::abs(rawLeftVel));
        double normalizationFactor = maxRawVel > 1.0 ? maxRawVel : 1.0;

        double leftVelMultiplier = rawLeftVel / normalizationFactor;
        double rightVelMultiplier = rawRightVel / normalizationFactor;

        int gearRatioMult = hw->getGearRatioMultiplier(hw->driveTrain);

        double leftVel =  100 * leftVelMultiplier * gearRatioMult;
        double rightVel =  100 * rightVelMultiplier * gearRatioMult;

        return {leftVel, rightVel};
}


void Drive::moveDriveTrain(std::pair<double,double> velPercent)
{
    std::pair<double,double> vel = calculateDriveTrainVel(velPercent);
    if(mirrorDrive > 0){
        hw->leftWheels.spin(vex::directionType::fwd, vel.first, vex::velocityUnits::rpm);
        hw->leftWheels.spin(vex::directionType::fwd, vel.second, vex::velocityUnits::rpm);
    }else{
        hw->leftWheels.spin(vex::directionType::rev, vel.first, vex::velocityUnits::rpm);
        hw->leftWheels.spin(vex::directionType::rev, vel.second, vex::velocityUnits::rpm);
    }
}


void Drive ::moveDriveTrainDistance(std::pair<double,double> velPercent, double distance) {
    double numberWheelRevolutions = distance / WHEELCIRCUMFERENCE;
    
    std::pair<double, double> vel = calculateDriveTrainVel(velPercent);
    hw->leftWheels.spinTo(numberWheelRevolutions * 360, vex::degrees, vel.first, vex::velocityUnits::rpm);
    hw->rightWheels.spinTo(numberWheelRevolutions * 360, vex::degrees, vel.second, vex::velocityUnits::rpm);
    vex::wait(50, vex::timeUnits::msec);
    while(hw->leftWheels.velocity(vex::velocityUnits::rpm) > 0 || hw->leftWheels.velocity(vex::velocityUnits::rpm));
}

void Drive :: spinIntake(bool ISSTOP, bool ISINVERT) {
    if (ISSTOP) {
        hw->intake.stop();
    }else {
        if(ISINVERT){
            hw->intake.spin(vex::directionType::rev, 12, vex::voltageUnits::volt);
        }else{
            hw->intake.spin(vex::directionType::fwd, 12, vex::voltageUnits::volt);
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
    int gearRatioMult = hw->getGearRatioMultiplier(hw->launcher);
    double velPercent = 100;
    hw->launcher.spin(vex::directionType::fwd, velPercent * gearRatioMult, vex::percentUnits::pct);
    vex::wait(150, vex::timeUnits::msec);
    hw->launcher.spin(vex::directionType::rev, velPercent * gearRatioMult, vex::percentUnits::pct);
    vex::wait(170, vex::timeUnits::msec);
    
}


void Drive::expand()
{
    int gearRatioMult = hw->getGearRatioMultiplier(hw->expansion);
    double velPercent = 100;

    hw->expansion.spin(vex::directionType::fwd, velPercent * gearRatioMult, vex::percentUnits::pct);
    vex::wait(150, vex::timeUnits::msec);
    hw->expansion.stop();
    vex::wait(170, vex::timeUnits::msec);

}