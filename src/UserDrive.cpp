#include "UserDrive.h"

UserDrive::UserDrive(Hardware *hardware, Telemetry *telemetry)
{
    hw = hardware;
    telemetry = tm;
    mp = new Map();
}

void UserDrive::drive()
{
    mirrorDriveToggle();
    driveTrainControls();
    intakeControls();
    flywheelControls();
    flickDiskControls();
    expandControls();
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

void UserDrive::flywheelControls(){
    if(hw->controller.ButtonB.pressing()){
        flywheelVoltage = (flywheelVoltage >= 8000) ? 8000 : 12000;
    }
    if(hw->controller.ButtonR1.pressing()){
       spinFlywheel(flywheelVoltage);
    }else if(hw->controller.ButtonY.pressing()){
        spinFlywheel(-12000);
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