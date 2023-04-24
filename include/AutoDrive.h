#pragma once
#include "Drive.h"
#include "vex.h"


class AutoDrive : public Drive
{
public:
    AutoDrive(Hardware* hardware, RobotConfig* robotConfig, Telemetry* telemetry);
    void drive();

private:
    vex::vision::signature yellow_disk = vex::vision::signature(1, -715,-291, -503, -4109, -3709, -3909, 5.7, 0);

    bool IS_USING_GPS_HEADING = false;
    bool IS_USING_GPS_POSITION = false;
    bool IS_USING_INERTIA_HEADING = false;
    bool IS_USING_ENCODER_POSITION = true; //If true, requires you to use tm->setManualPosition({x,y}) before you call autoDrive functions
    bool IS_USING_ENCODER_HEADING = true;   //If true, requires you to use tm->setManualHeading(heading) before you call autoDrive functions

    /// @brief Shoots after desired velocity is reached 
    /// @param velocity The desired velocity 
    void shootAtDesiredVelocity(double velocityRPM, int numFlicks);

    /// @brief Uses drivetrain ENCODERS to rotate to a relativ angle
    /// @param angle Angle in degrees (-360 to 360) where positive angle is counterclockwise rotation
    void rotateToRelativeAngle(double angle);

    /// @brief  Rotates the shortest distance by turning left or right to the heading. Assumes the inertia sensor is set so 0 is at the positive x axis.
    /// @param heading  Double that is the counterclockwise angle in degrees from the x asis.
    void rotateToHeading(double heading);

    /// @brief   Rotates the robot to align with the element on the field. Assumes the inertia sensor is set so 0 is at the positive x axis.
    /// @param gameElement  GameElement that is the object to rotate to.
    void rotateToPosition(GameElement* gameElement);
    
    /// @brief  Rotates the robot to align with a coordinate on the field. Assumes the inertia sensor is set so 0 is at the positive x axis.
    /// @param finalPosition    Pair of doubles {x,y} of the coordinate in inches to align with {0,0} being the center of the field
    /// @param ISBACKROTATION   Boolean that if true, rotates the back of the robot to the coordinate instead of the front
    void rotateToPosition(std::pair<double,double> finalPosition, bool ISBACKROTATION = false);

    /// @brief  Rotates the robot to align with a GameElement on the field. Assumes the inertia sensor is set so 0 is at the positive x axis.
    /// @param gameElement  GameElement that is the object to rotate and drive to.
    void rotateAndDriveToPosition(GameElement* gameElement);


    /// @brief  Rotates the robot to align with a coordinate on the feild and drives to that position. 
    ///         Assumes the inertia sensor is set so 0 is at the positive x axis.
    /// @param position             Pair of doubles {x,y} of the coordinate in inches to align and drive to with {0,0}
    ///                             being the center of the field.
    /// @param ISBACKTOPOSITION     Boolean that if true, rotates the back of the robot to the coordinate instead of the front.
    void rotateAndDriveToPosition(std::pair<double,double> position, bool ISBACKTOPOSITION = false);


    /// @brief          Rotates to a goal and shoots disk(s).
    /// @param goal     GameElement that is the goal to shoot at (42 or 43)/
    /// @param velocityPercent  Double percent from 0 to 100 to shoot the disk(s) at.
    /// @param numDisksToShoot  Integer number of disks in the hopper to shoot (1-3).
    void rotateAndShoot(GameElement* goal, double velocityPercent, int numDisksToShoot);

    void rollRoller(vex::color ourColor);

    void usePathing(); 

    

    void q2RedPathAlgo(vex::color ourColor, bool isSkills); //Granny

    void q3BluePathAlgo(vex::color ourColor, bool isSkills);

    void q4RedPathAlgo(vex::color ourColor, bool isSkills);    //Sid

    void q2BluePathAlgo(vex::color ourColor, bool isSkills);   //Sid
    
    void q4BluePathAlgo(vex::color ourColor, bool isSkills);   //Granny

    void skillsPathing(vex::color ourColor, bool isSkills);

    /// @brief Centers the robot on the biggest yellow blob using the vision sensor.
    void centerOnDisk();

};