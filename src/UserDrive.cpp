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
    //intakeControls();
    flyweelControlswPID();
    flickDiskControls();
    expandControls();
    hw->controller.Screen.clearScreen();
    hw->controller.Screen.setCursor(1, 1);
    hw->controller.Screen.print("T: %.2f B: %.2f", hw->flywheelTop.velocity(vex::rpm), hw->flywheelBottom.velocity(vex::rpm));
    hw->controller.Screen.newLine();
    vex::wait(150, vex::timeUnits::msec);
    hw->controller.Screen.print("kp: %.2f Out: %.2f", Kp ,output);
    hw->controller.Screen.newLine();
    vex::wait(150, vex::timeUnits::msec);
   // hw->controller.Screen.newLine();
    hw->controller.Screen.print("Ki: %.4f Kd: %.4f", Ki, Kd);
    vex::wait(150, vex::timeUnits::msec);

    //hw->controller.Screen.clearScreen();
    ///hw->controller.Screen.setCursor(1, 1);
    //vex::wait(100, vex::timeUnits::msec);
}

void UserDrive::driveTrainControls(){
    const int DEADZONE = 2;

    double forwardBackward = (double) hw->controller.Axis3.position(vex::percentUnits::pct);
    double turning = (double) hw->controller.Axis1.position(vex::percentUnits::pct);

    if(std::abs(forwardBackward) < DEADZONE){
        forwardBackward = 0;
    }


    if(std::abs(turning) < DEADZONE) {
        turning = 0;
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
    if(hw->controller.ButtonB.pressing()){
        if(hw->controller.ButtonLeft.pressing()){
            Kp+=1.0;
        }

        if(hw->controller.ButtonRight.pressing()){
            Kp-=1.0;
        }

        if(hw->controller.ButtonUp.pressing()){
            Kp+=0.01;
        }

        if(hw->controller.ButtonDown.pressing()){
            Kp-=0.01;
        }
    }else if(hw->controller.ButtonX.pressing()){

        if(hw->controller.ButtonLeft.pressing()){
            Ki+=1.0000;
        }

        if(hw->controller.ButtonRight.pressing()){
            Ki-=1.0000;
        }

        if(hw->controller.ButtonUp.pressing()){
            Ki+=0.01;
        }

        if(hw->controller.ButtonDown.pressing()){
            Ki-=0.01;
        }
    }else if(hw->controller.ButtonY.pressing()){

        if(hw->controller.ButtonLeft.pressing()){
            Kd+=0.01;
        }

        if(hw->controller.ButtonRight.pressing()){
            Kd-=0.01;
        }

        if(hw->controller.ButtonUp.pressing()){
            Kd+=0.001;
        }

        if(hw->controller.ButtonDown.pressing()){
            Kd-=0.001;
        }
    }

    
    // PID variables
    if(hw->controller.ButtonR1.pressing())
    {
        double maxRPM = 600;
        double targetRPM = flywheelVoltage / 12.0 * maxRPM;
        double currentRPM = (hw->flywheelTop.velocity(vex::rpm) + hw->flywheelBottom.velocity(vex::rpm)) / 2;// = (flywheelTopMotor.velocity(rpm) + flywheelBottomMotor.velocity(rpm)) / 2;

        // PID calculations
        this->error = targetRPM - currentRPM;
        this->error = this->error / maxRPM * 12;
        this->integral += error;
        this->derivative = error - prevError;
        this->output = Kp * error + Ki * integral + Kd * derivative;
        this->prevError = error;
        //this->outPutVolt = output / maxRPM * 12;

        // Limit output to valid motor voltage
        if (output > 12.0)
            output = 12.0;
        if (output < -12.0)
            output = -12.0;

        spinFlywheel(output);

        //hw->controller.Screen.print("Kp:%.2f Output: %d", Kp, output);
        //vex::wait(100, vex::timeUnits::msec);
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