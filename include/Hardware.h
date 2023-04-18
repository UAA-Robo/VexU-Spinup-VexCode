#pragma once
#include "vex.h"

class Hardware{

public:
    //Constructors
    Hardware();

    int getGearRatioMultiplier(vex::motor &motor); 
    int getGearRatioMultiplier(vex::motor_group &motorGroup);  

    vex::brain brain;
    vex::controller controller = vex::controller(vex::primary);

    vex::inertial inertiaSensor = vex::inertial(vex::PORT19);
    vex::distance distanceSensor = vex::distance(vex::PORT11);
    vex::vision visionSensor = vex::vision(vex::PORT12);
    vex::optical opticalSensor = vex::optical(vex::PORT20);
    vex::gps gpsSensor = vex::gps(vex::PORT18, /*originx*/ 0, /*originy*/ -3.3, /*dist unit*/ vex::distanceUnits::in, /*head offset*/ 180, /*direction turntype*/ vex::turnType::right); //SID Offset

    vex::motor wheelLeftFront = vex::motor(vex::PORT1, vex::ratio18_1, false);
    vex::motor wheelleftBack = vex::motor(vex::PORT2, vex::ratio18_1, true);
    vex::motor wheelRightFront = vex::motor(vex::PORT9, vex::ratio18_1, true);
    vex::motor wheelRightBack = vex::motor(vex::PORT10, vex::ratio18_1, false);

    vex::motor_group leftWheels = vex::motor_group(wheelLeftFront, wheelleftBack);
    vex::motor_group rightWheels = vex::motor_group(wheelRightFront, wheelRightBack);

    vex::motor_group driveTrain = vex::motor_group(wheelLeftFront, wheelleftBack, wheelRightFront, wheelRightBack);
    vex::smartdrive smartDriveTrain = vex::smartdrive(leftWheels, rightWheels, inertiaSensor);

    vex::motor flywheelTop = vex::motor(vex::PORT4, vex::ratio6_1, false);
    vex::motor flywheelBottom = vex::motor(vex::PORT5, vex::ratio6_1, true);
    vex::motor_group flywheel = vex::motor_group(flywheelTop, flywheelBottom); //Use this

    vex::motor launcher = vex::motor(vex::PORT3, vex::ratio18_1, false);    //Use this


    vex::motor sideExpansion = vex::motor(vex::PORT14,vex::ratio18_1);
    vex::motor frontExpansion = vex::motor(vex::PORT15,vex::ratio18_1);

    vex::motor_group expansion = vex::motor_group(sideExpansion, frontExpansion);

    // intake motors
    vex::motor intakeBackLeft = vex::motor(vex::PORT6, vex::ratio6_1, false);
    vex::motor intakeMiddlLeft = vex::motor(vex::PORT7, vex::ratio6_1, true);
    vex::motor intakeMiddleRight = vex::motor(vex::PORT8, vex::ratio6_1, false);
    vex::motor intakeTopRight = vex::motor(vex::PORT20, vex::ratio6_1, true);

    vex::motor_group intake = vex::motor_group(intakeBackLeft, intakeMiddlLeft, intakeMiddleRight, intakeTopRight); //Use this



};