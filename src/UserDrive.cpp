#include "UserDrive.h"

UserDrive::UserDrive(Hardware* hardware, RobotConfig* robotConfig, Telemetry* telemetry): Drive(hardware, robotConfig, telemetry) 
{}

int UserDrive::getFlywheelSpeed()
{
    return flywheelVoltage;
}

void UserDrive::drive()
{
    mirrorDriveToggle();
    driveTrainControls();
    intakeControls();
    flyweelControlswPID();
    flickDiskControls();
    expandControls();
    //hw->controller.Screen.print("%.3f", flywheelVoltage);
    vex::wait(100, vex::timeUnits::msec);
    hw->controller.Screen.clearLine();
    hw->controller.Screen.setCursor(1, 1);
    vex::wait(100, vex::timeUnits::msec);
}

void UserDrive::driveTrainControls(){
    const int DEADZONE = 5;

    double forwardBackward = (double) hw->controller.Axis3.position(vex::percentUnits::pct);
    double turning = (double) hw->controller.Axis1.position(vex::percentUnits::pct);

    if(std::abs(forwardBackward) < DEADZONE){
        forwardBackward = 0;
    }else if(forwardBackward < 0){
        forwardBackward += 5;
    }else{
        forwardBackward -= 5;
    }

    if(std::abs(turning) < DEADZONE) {
        turning = 0;
    }else if(turning < 0){
        turning +=5;
    }else{
        turning -= 5;
    }

    moveDriveTrain({forwardBackward, turning});
}

void UserDrive::intakeControls(){
    if(hw->controller.ButtonL1.pressing()){
        spinIntake(false, false);
    }else if(hw->controller.ButtonX.pressing()){
        spinIntake(false, true);
    }else{
        spinIntake(true, true);
    }
}
void UserDrive::flyweelControlswPID()
{
    double Kp = 0.15;
    double Ki = 0.000;
    double Kd = 0.00;

    // PID variables
    if(hw->controller.ButtonY.pressing())
    {
    double maxRPM = 600;
    double targetRPM = flywheelVoltage / 12.0 * maxRPM;
    double currentRPM = (hw->flywheelTop.velocity(vex::rpm) + hw->flywheelBottom.velocity(vex::rpm)) / 2;// = (flywheelTopMotor.velocity(rpm) + flywheelBottomMotor.velocity(rpm)) / 2;

    // PID calculations
    this->error = targetRPM - currentRPM;
    this->integral += error;
    this->derivative = error - prevError;
    this->output = Kp * error + Ki * integral + Kd * derivative;
    this->prevError = error;

    // Limit output to valid motor voltage
    if (output > 12.0)
        output = 12.0;
    if (output < -12.0)
        output = -12.0;

    spinFlywheel(output);
    }
    else
    {
        spinFlywheel(0);
    }
}

void UserDrive::flywheelControls(){

    // if(hw->controller.ButtonDown.pressing()){
    //     flywheelVoltage-=0.1;
    // }

    // if(hw->controller.ButtonUp.pressing()){
    //     flywheelVoltage+=0.1;
    // }

    if(hw->controller.ButtonB.pressing()){
        flywheelVoltage = (flywheelVoltage >= 8) ? 8 : 12;
    }

    //flywheelVoltage = 7.4;
    if(hw->controller.ButtonR1.pressing()){
       spinFlywheel(flywheelVoltage);
    }else if(hw->controller.ButtonY.pressing()){
        spinFlywheel(-12);
    }else{
        spinFlywheel(0);
    }
}

void UserDrive::flickDiskControls(){
    if(hw->controller.ButtonR2.pressing()){
        flickDisk();
    }
}

void UserDrive::expandControls(){
    if(hw->controller.ButtonA.pressing()){
        expand();
    }
}

void UserDrive::mirrorDriveToggle(){
    mirrorDrive = (hw->controller.ButtonL2.pressing()) ? -1 : 1;
}