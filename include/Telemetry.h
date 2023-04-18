
#pragma once
#include "Hardware.h"
#include "vex.h"

class Telemetry {

public:
    Telemetry(Hardware* devices, double DRIVETRAINWHEELCIRCUMFERENCE);

    /// @brief              Calculates the distance (in inches) between to points
    /// @param initPos      Pair of doubles {x, y} that represent the first coordinate (where the origin is the center of the field)
    /// @param finalPos     Pair of doubles {x, y} that represent the second coordinate (where the origin is the center of the field)
    /// @return             Returns the distance in inches
    double getDistanceBtwnPoints(std::pair<double,double> initPos, std::pair<double,double> finalPos);

    /// @brief              Calculates the counterclockwise angle from the x axis (of the field) to the line formed from by 2 points.
    /// @param initPos      Pair of doubles {x, y} that represent the first coordinate (where the origin is the center of the field)
    /// @param finalPos     Pair of doubles {x, y} that represent the second coordinate (where the origin is the center of the field)
    /// @return             Returns angle in degrees.
    double getHeadingBtwnPoints(std::pair<double,double> initPos, std::pair<double,double> finalPos);

    /// @brief      Calculates the displacement that the robot has moved in inches based on encoder positions. Assumes 
    ///             the motors were reset at the beginning of the displacement disstance
    /// @param WHEELCIRCUMFERENCE   Double that is the circumference of the drive train wheels in inches
    /// @return     Returns the displacement in inches
    double getDrivetrainEncoderDisplacement(); 
    
    /// @brief      Updates and returns the distance the robot has moved in inches based on encoder positions. Assumes 
    ///             the motors were reset at the beginning of the displacement disstance   
    /// @param WHEELCIRCUMFERENCE   Double that is the circumference of the drive train wheels in inches
    /// @return     Returns that distance in inches
    double updateDrivetrainEncoderTotalDistance();
    
    /// @brief      Calculates the displacement that the robot has moved in inches based on gps positions.
    /// @param initPosition   Pair of doubles {x, y} that represent the first coordinate
    /// @return     Returns the displacement in inches
    double getGpsDisplacement(std::pair<double,double> initPosition);

    /// @brief      Updates and returns the distance the robot has moved in inches based on the gps values.
    /// @param lastUpdatedPosition      Pair of doubles {x, y} that represents the last final coordinate used to update 
    ///                                 gpsTotalDistance the last time it was updated
    /// @return     Returns that distance in inches
    double updateGpsTotalDistance(std::pair<double,double> lastUpdatedPosition);


    /// @brief      Calculates the linear x and y velocity in in relation to the feild. Uses the motor encoders for
    ///             the wheel rotational velocity and the inertia seensor for heading of the robot. Assumes the origin is considered 
    ///             the center of the feild and the inertial sensor heading was initialized with the pos x axis being 0.
    ///             Math based on https://www.cs.cmu.edu/~rasc/Download/AMRobots3.pdf
    /// @param  WHEELCIRCUMFERENCE Double that is the circumference of the drive train wheels in inches
    /// @return     Returns a pair of doubles {xLinearVelocity, yLinearVelocity} in inches per minute.
    std::pair<double,double> getLinearVel();


    /// @brief      Calculates the current GPS position by taking a snapshot of GPS values using snapShotAverage()
    /// @return     Returns a pair of doubles {x, y} that represents the last current coordinate (where the origin is the center of the field)
    std::pair<double,double> getGPSPosition();

    /// @brief      Calculates the current gps heading (positive counterclockwise from the x-axis) by taking a snapshot of inertia values using snapShotAverage()
    /// @return     Returns a double represents the current heading in degrees (where the origin positive x axis) from 0-360
    double getGPSHeading();

    /// @brief      Calculates the current inertia heading (positive counterclockwise from the x-axis) by taking a snapshot of inertia values using snapShotAverage()
    /// @return     Returns a double represents the current heading in degrees (where the origin positive x axis) from 0-360
    double getInertiaHeading();

    /// @brief      Sets the current inertia sensor heading to the current GPS heading
    void setInertiaHeadingToGPS();

private:
    Hardware* hw;
    double gpsTotalDistance; 
    double drivetrainEncoderTotalDistance;
    const int SNAPSHOTSIZE = 1600;
    double WHEELCIRCUMFERENCE;

    /// @brief              For a snapshot of data (size SNAPSHOTSIZE), disccards the outliers using boxplot statistics and takes the average of the data.
    /// @param snapshot     Vector of doubles of size SNAPSHOTSIZE that contains the data points to be normalized and averaged.
    /// @return             Double of thhe normalized/averaged data.
    double snapShotAverage(std::vector<double> snapshot);
    std::pair<double,double> snapShotAverage(std::vector<std::pair<double,double>>& snapshot);
    
};

