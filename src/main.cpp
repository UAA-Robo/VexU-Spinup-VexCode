/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       wwami                                                     */
/*    Created:      4/18/2023, 12:20:06 AM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "Robot.h"

using namespace vex;

// A global instance of competition
competition Competition;
Robot* icebot;
bool isControlled;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {

  icebot = new Robot();
  //vex::wait(2000, vex::msec);
  
  return;
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/


void autonomous(void) {
  icebot = new Robot();
 // if(!isControlled){
    icebot->driveAuto();
  //}
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/


void usercontrol(void) {
  icebot = new Robot();
  while (true) {
    icebot->drive();
    vex::wait(20, vex::msec); // Sleep the task for a short amount of time to prevent wasted resources.
  }
}


//
// Main will set up the competition functions and callbacks.
//
int main() {
  isControlled = true;
  
  // Set up callbacks for autonomous and driver control periods.

  //icebot = new Robot();
  // Run the pre-autonomous function.
  pre_auton();

  //Competition.autonomous(autonomous);
  //Competition.drivercontrol(usercontrol);

  //TESTING
  autonomous();
  usercontrol();


  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
