#pragma once
#include "vex.h"
#include "Hardware.h"
#include "Telemetry.h"
#include "RobotConfig.h"
#include "map/Map.h"
#include "Logger.h"

class Drive{


public:
    virtual void drive(){}

    /// @brief Outputs distance from robot to goal to computer screen
    double outputDistanceToGoal();

    void flyweelControlswPID();


protected:
    Drive(Hardware* hardware, RobotConfig* robotConfig, Telemetry* telemetry);

    Hardware* hw;
    RobotConfig* rc;
    Telemetry* tm;
    Map* mp;
    Logger* positionLog;

    std::pair<double, double> positionGoal; //For logging thread
    double headingGoal; //For logging purposes

    bool ERROR_CORRECTION_ENABLED = false; //used for positionErrorCorrect and headingErrorCorrect

    double outPutVolt;
    double Kp = 0.9;
    double Ki = 0.3;
    double Kd = 0.6;
    

    /// @brief Adds data to log.
    /// @return Tasks must return an int so this returns 0 (but it dosn't mean anything)
    static int logData(void* param);

    /// @brief      Calculates the velocity in RPMs that the left and right drivetrain wheels should recieve based on
    ///             the horizontal percentage and vertical percentage passed in. Automatically scales the velocities
    ///             for the drivetrain gear inserts.
    /// @param velPercent   Pair of doubles {verticalVelocityPercent, horizontalVelocityPercent} from -100 to 100 
    ///                     that represent the percentage that the drivetrain should move forward/backward and left/right.
    ///                     For example of verticalVelocityPercent = 50  and horizontalVelocityPercent = 0, the bot 
    ///                     will move forward at 50% velocity. Likewise if verticalVelocityPercent = 0  and 
    ///                     horizontalVelocityPercent = 50, the drivetrain will rotate to the right at 50% velocity.
    ///                     Any combiniatiion of non-zero verticalVelocityPercents and horizontalVelocityPercents 
    ///                     will cause the drivetrain to move in a arc.
    /// @return     Returns a pair of doubles {leftWheelsVelocity, rightWheelsVelocity} that represent the actual velocities in RPM 
    ///             (scaled to the gear-ratio) that the wheels on each drivetrain side need to be set to in order move as expected. 
    std::pair<double,double>  calculateDriveTrainVel(std::pair<double,double> velPercent); 

    /// @brief       Moves the drivetrain based on the horizontal percentage and vertical percentage passed in. Calls calculateDriveTrainVel
    ///             to convert those percentages to actual velocites in RPM.
    /// @param velPercent   Pair of doubles {verticalVelocityPercent, horizontalVelocityPercent} from -100 to 100 
    ///                     that represent the percentage that the drivetrain should move forward/backward and left/right.
    ///                     For example of verticalVelocityPercent = 50  and horizontalVelocityPercent = 0, the bot 
    ///                     will move forward at 50% velocity. Likewise if verticalVelocityPercent = 0  and 
    ///                     horizontalVelocityPercent = 50, the drivetrain will rotate to the right at 50% velocity.
    ///                     Any combiniatiion of non-zero verticalVelocityPercents and horizontalVelocityPercents 
    ///                     will cause the drivetrain to move in a arc.
    void moveDriveTrain(std::pair<double,double> velPercent);

    /// @brief      Moves the drivetrain a cetain distance based on the horizontal percentage and vertical percentage passed in. Calls calculateDriveTrainVel
    ///             to convert those percentages to actual velocites in RPM.
    /// @param velPercent   Pair of doubles {verticalVelocityPercent, horizontalVelocityPercent} from -100 to 100 
    ///                     that represent the percentage that the drivetrain should move forward/backward and left/right.
    ///                     For example of verticalVelocityPercent = 50  and horizontalVelocityPercent = 0, the bot 
    ///                     will move forward at 50% velocity. Likewise if verticalVelocityPercent = 0  and 
    ///                     horizontalVelocityPercent = 50, the drivetrain will rotate to the right at 50% velocity.
    ///                     Any combiniatiion of non-zero verticalVelocityPercents and horizontalVelocityPercents 
    ///                     will cause the drivetrain to move in a arc.
    /// @param distance     Distance in inches to move.
    void moveDriveTrainDistance(std::pair<double,double> velPercent, double distance);

    /// @brief  Spins the intake indefinitely unless ISSTOP is passed as true.
    /// @param ISSTOP       If true, stops the intake.
    /// @param ISINVERT     If true, runs the intake in reverse;
    void spinIntake(bool ISSTOP = false, bool ISINVERT = false, int volts=12);

    //void (Drive::*spinIntakePtr)(bool, bool) = &Drive::spinIntake;

    /// @brief  Sets the voltage and spins the flywheel until the function is called again with a differrent voltage (like 0).
    /// @param voltage      Double from -12,000 to 12,000. Negative voltage reversed the flywheel.
    void spinFlywheel(double voltage);

    /// @brief Flicks the disk by moving the launcer/flicker forward and backward once.
    void flickDisk();

    /// @brief  Expands the bot by launching string by pulling the pins from 3 catapults.
    void expand();
    
    /// @brief TODO: NOT implemented yet!!!!
    void errorCorrect();
    

};

