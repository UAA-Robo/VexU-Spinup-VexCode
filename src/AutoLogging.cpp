#pragma once
#include "vex.h"
#include "Hardware.h"
#include "Telemetry.h"
#include "Drive.h"
#include "Logger.h"


// class AutoLogging {

// public:
//     AutoLogging(Hardware* hardware, Telemetry* telemetry, Drive* drive) {
//     hw = hardware;
//     tm = telemetry;
//     dr = drive;
// }

//     void startLogging() {
//         if (isRunning) return;
//         // Start a new task to run the data logging function
//         logTask = vex::task(logData, this);
//         isRunning = true;
//     }

//     void stopLogging() {
//         if (!isRunning) return;
        
//         // Stop the logging task
//         logTask.stop();
//         isRunning = false;
//     }

// private:
//     bool isRunning;

//     Logger positionLog* = new Logger(hw, "positionData.txt", {"initX", "initY", "goalX", "goalY", "gpsX", "gpsY", 
//                                                     "initHeading","goalHeading", "gpsHeading", "rotationSensorHeading",
//                                                     "wheelDisplacementFL", "wheelDisplacementFR","wheelDisplacementBL","wheelDisplacementBR",
//                                                     "wheelVelFL", "wheelVelFR","wheelVelBL","wheelVelBR",
//                                                     "wheelVoltFL", "wheelVolFR","wheelVolBL","wheelVolBR",
//                                                     "wheelPowerFL", "wheelPowerFR","wheelPowerBL","wheelPowerBR",
//                                                     });


//     /* 
//     X and Y coords are in inches
//     Heading is in degrees from the positive x axis, rotated counterclockwise
//     Wheel Displacement  is in revolutions (FL = Front Left, FR = Front Right, BL = Back Left, BR = Back Front)
//     Wheel Velocity (Vel) is in RPM (
//     Wheel Voltage is in volts
//     Wheel Power is in watts
//     */
//     int logData() {
        
//         positionLog->addData({tm->getCurrPosition().first, tm->getCurrPosition().second, positionGoal.first, positionGoal.second,tm->getGPSPosition().first, tm->getGPSPosition().second, 
//                                 tm->getCurrHeading(), headingGoal, tm->getGPSHeading(), tm->getInertiaHeading(),
//                                 hw->wheelLeftFront.position(vex::rev), hw->wheelRightFront.position(vex::rev), hw->wheelLeftBack.position(vex::rev),hw->wheelRightBack.position(vex::rev),
//                                 hw->wheelLeftFront.velocity(vex::rpm), hw->wheelRightFront.velocity(vex::rpm), hw->wheelLeftBack.velocity(vex::rpm),hw->wheelRightBack.velocity(vex::rpm),
//                                 hw->wheelLeftFront.voltage(vex::volt), hw->wheelRightFront.voltage(vex::volt), hw->wheelLeftBack.voltage(vex::volt),hw->wheelRightBack.voltage(vex::volt),
//                                 hw->wheelLeftFront.power(vex::powerUnits::watt), hw->wheelRightFront.power(vex::powerUnits::watt), hw->wheelLeftBack.power(vex::powerUnits::watt),hw->wheelRightBack.power(vex::powerUnits::watt),
//                             });

//         vex::wait(1, vex::sec);
        

//         return 0;
//     }


// };





