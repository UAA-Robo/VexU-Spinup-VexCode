#include "Drive.h"

Drive::Drive(Hardware* hardware, RobotConfig* robotConfig, Telemetry* telemetry) {
    hw = hardware;
    rc = robotConfig;
    tm = telemetry;
    mp = new Map();
    tm->setCurrPosition({0,0});
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
    while(hw->leftWheels.velocity(vex::velocityUnits::pct) > 0 || hw->leftWheels.velocity(vex::velocityUnits::pct)); //Blocks other tasks from starting 
    
    double currHeading = tm->getCurrHeading();
    double newX = tm->getCurrPosition().first + distance * cos(currHeading * M_PI / 180.0); //need to convert degrees to radians
    double newY = tm->getCurrPosition().second + distance * sin(currHeading * M_PI / 180.0);
    tm->setCurrPosition({newX, newY});
    tm->headingErrorCorrection();
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