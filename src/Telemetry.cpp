
#include "Telemetry.h"



Telemetry ::Telemetry(Hardware* hardware, RobotConfig* robotConfig)
{
    hw = hardware;
    rc = robotConfig;

    gpsTotalDistance = 0;
    drivetrainEncoderTotalDistance = 0;
}

void Telemetry::printGPSInertiaData() {
    hw->controller.Screen.clearScreen();

    hw->controller.Screen.setCursor(1,1);
    hw->controller.Screen.print("gps x: %.1f y: %.1f", getGPSPosition().first, getGPSPosition().second);
    vex::wait(200,vex::msec);

    hw->controller.Screen.setCursor(2,1);
    hw->controller.Screen.print("gps heading: %.1f", getGPSHeading());
    vex::wait(200,vex::msec);

    hw->controller.Screen.setCursor(3,1);
    hw->controller.Screen.print("inertia heading: %.1f", getInertiaHeading());
    vex::wait(200,vex::msec);
    
}


double Telemetry ::getDistanceBtwnPoints(std::pair<double, double> initPos, std::pair<double, double> finalPos)
{
    double distanceToFinalPosition = sqrt(pow((finalPos.first - initPos.first), 2) + pow((finalPos.second - initPos.second), 2));
    return distanceToFinalPosition;
}

double Telemetry ::getHeadingBtwnPoints(std::pair<double, double> initPos, std::pair<double, double> finalPos)
{
    double angleToFinalPosition = (atan2((finalPos.second - initPos.second), (finalPos.first - initPos.first)) * (180 / (M_PI)));
    return angleToFinalPosition;
}

double Telemetry ::getDrivetrainEncoderDisplacement()
{
    double avgEncoderRotations = 0; // in revolutions
    double displacement;
    int numEncoders = hw->driveTrain.count();

    // averag encoders
    avgEncoderRotations += hw->wheelLeftBack.position(vex::rev);
    avgEncoderRotations += hw->wheelLeftFront.position(vex::rev);
    avgEncoderRotations += hw->wheelRightBack.position(vex::rev);
    avgEncoderRotations += hw->wheelRightFront.position(vex::rev);

    avgEncoderRotations /= numEncoders;

    // calulate displacement
    displacement = avgEncoderRotations * rc->WHEELCIRC;

    return displacement;
}

double Telemetry ::updateDrivetrainEncoderTotalDistance()
{
    drivetrainEncoderTotalDistance += getDrivetrainEncoderDisplacement();
    return drivetrainEncoderTotalDistance;
}

double Telemetry ::getGpsDisplacement(std::pair<double, double> initPosition)
{
    return getDistanceBtwnPoints(initPosition, getGPSPosition());
}

double Telemetry ::updateGpsTotalDistance(std::pair<double, double> lastUpdatedPosition)
{
    gpsTotalDistance += getGpsDisplacement(lastUpdatedPosition);
    return gpsTotalDistance;
}

std::pair<double, double> Telemetry::getLinearVel()
{
    double avgLeftRotationalVel = 0;  // Motors on left side of drivetrain
    double avgRightRotationalVel = 0; // Motors on right side of drivetrain
    int numEncoders = hw->leftWheels.count(); // Assume left and right side have same number of mootors


    avgLeftRotationalVel += hw->wheelLeftBack.velocity(vex::pct);
    avgLeftRotationalVel += hw->wheelLeftFront.velocity(vex::pct);
    avgRightRotationalVel += hw->wheelRightBack.velocity(vex::pct);
    avgRightRotationalVel += hw->wheelRightFront.velocity(vex::pct);

    avgLeftRotationalVel /= numEncoders;
    avgRightRotationalVel /= numEncoders;

    double angleFromXAxis = getInertiaHeading() * M_PI / 180; // converted to rads for cos() and sin()
    const double WHEELDIAMETER = rc->WHEELCIRC / M_PI;

    // Inches per minute
    double xLinearVel = cos(angleFromXAxis) * WHEELDIAMETER * (avgLeftRotationalVel + avgRightRotationalVel) / 2; // In reference to the field
    double yLinearVel = sin(angleFromXAxis) * WHEELDIAMETER * (avgLeftRotationalVel + avgRightRotationalVel) / 2; // In reference to the field

    return {xLinearVel, yLinearVel};
}

std::pair<double, double> Telemetry::getGPSPosition()
{
    std::vector<double> xSnapshot;
    std::vector<double> ySnapshot;

    std::pair<double, double> currPosition;

    for (int i = 0; i < rc->SNAPSHOTSIZE ; ++i)
    {
        xSnapshot.push_back(hw->gpsSensor.xPosition(vex::inches));
        ySnapshot.push_back(hw->gpsSensor.yPosition(vex::inches));
    }

    currPosition.first = snapShotAverage(xSnapshot);
    currPosition.second = snapShotAverage(ySnapshot);

    // std::pair<double,double> currPosition = {hw->gps.get_status().x, hw->gps.get_status().y};
    return currPosition;
}

double Telemetry::getGPSHeading()
{
    std::vector<double> snapshot;
    double heading;

    for (int i = 0; i < rc->SNAPSHOTSIZE; ++i)
    {
        
        heading = hw->gpsSensor.heading();
        
        // Correct heading to be always be positive counterclockwise from the x axis bc the gps sensor reads clockwise form the positive y axis
        heading = fmod(90 - heading, 360.0); 
        heading = (heading >= 0 ? heading : heading + 360.0);

        snapshot.push_back(heading);
    }

    return snapShotAverage(snapshot);
}

double Telemetry::getInertiaHeading()
{
    std::vector<double> snapshot;
    double heading;

    for (int i = 0; i < rc->SNAPSHOTSIZE; ++i)
    {

        heading = hw->inertiaSensor.heading();

        // Correct heading to be always be positive counterclockwise from the x axis bc the inertia sensor reads clockwise form the x axis
        heading = fmod(360.0 - heading, 360.0);
        heading = (heading >= 0 ? heading : heading + 360.0);

        snapshot.push_back(heading);
    }

    return snapShotAverage(snapshot);
}

void Telemetry::setInertiaHeadingToGPS()
{
    double heading = getGPSHeading();
    // Changes heading from counterclockwise from the x axis (as returned by getGPSHeading) to clockwise because inertia sensor reads clockwise
    heading = fmod(360.0 - heading, 360.0);
    heading = (heading >= 0 ? heading : heading + 360.0);

    hw->inertiaSensor.setHeading(heading, vex::degrees);
}

void Telemetry::setCurrPosition(std::pair<double, double> currPos)
{
    this->currentPosition = currPos;
}

void Telemetry::positionErrorCorrection(double errorBounds)
{
    std::pair<double,double> gpsEstimate = getGPSPosition();
    int x = 0;
    while((fabs(gpsEstimate.first - currentPosition.first) > errorBounds || fabs(gpsEstimate.second - currentPosition.second) > errorBounds) && x < 3)
    {
        gpsEstimate = getGPSPosition();
        ++x;
    }

    if (x == 3)
    {
        hw->controller.Screen.setCursor(2,1);
        hw->controller.Screen.print("Used Encoder Value");
        return;
    }
    hw->controller.Screen.setCursor(2,1);
    hw->controller.Screen.print("Used GPS Value");
    this->currentPosition = gpsEstimate;
}

std::pair<double, double> Telemetry::getCurrPosition()
{
    return this->currentPosition;
}



void Telemetry::setCurrHeading(double currHeading)
{
    this->currHeading = currHeading;
}

void Telemetry::headingErrorCorrection(double errorBounds)
{
    const int numChecks = 3;
    double gpsEstimate = getGPSHeading();
    int x = 0;
    while(fabs(gpsEstimate - currHeading) > errorBounds && x < numChecks)
    {
        gpsEstimate = getGPSHeading();
        ++x;
    }

    if (x == numChecks)
    {
        hw->controller.Screen.setCursor(3,1);
        hw->controller.Screen.print("Used Encoder Heading");
        return;
    }
    hw->controller.Screen.setCursor(3,1);
    hw->controller.Screen.print("Used GPS Heading");
    this->currHeading = gpsEstimate;
}

double Telemetry::getCurrHeading()
{
    return this->currHeading;
}

double Telemetry::snapShotAverage(std::vector<double> snapshot)
{
    if (snapshot.empty())
    {
        return 0.0;
    }

    std::sort(snapshot.begin(), snapshot.end());

    size_t dataSize = snapshot.size();
    size_t q1Idx, q2Idx, q3Idx;
    double quartile1, quartile2, quartile3, IQR;

    // Find quartile indices
    q2Idx = dataSize / 2;
    q1Idx = q2Idx / 2;
    q3Idx = (dataSize % 2 == 0) ? (q2Idx + q1Idx) : (q2Idx + q1Idx + 1);

    // Calculate quartiles
    quartile1 = (q2Idx % 2 == 0) ? (snapshot[q1Idx - 1] + snapshot[q1Idx]) / 2.0 : snapshot[q1Idx];
    quartile2 = (dataSize % 2 == 0) ? (snapshot[q2Idx - 1] + snapshot[q2Idx]) / 2.0 : snapshot[q2Idx];
    quartile3 = (q2Idx % 2 == 0) ? (snapshot[q3Idx - 1] + snapshot[q3Idx]) / 2.0 : snapshot[q3Idx];

    IQR = quartile3 - quartile1;

    // Replace outliers with corresponding quartiles
    for (size_t i = 0; i < dataSize; ++i)
    {
        if (snapshot[i] < quartile1 - (1.5 * IQR))
        {
            snapshot[i] = quartile1;
        }
        else if (snapshot[i] > quartile3 + (1.5 * IQR))
        {
            snapshot[i] = quartile3;
        }
    }

    // Calculate average
    double sum = std::accumulate(snapshot.begin(), snapshot.end(), 0.0);
    double avg = sum / dataSize;

    return avg;
}

std::pair<double, double> Telemetry::snapShotAverage(std::vector<std::pair<double, double>>& snapshot)
{
    if (snapshot.empty())
    {
        return {0.0,0.0};
    }
    std::vector<double> snapshotx;
    std::vector<double> snapshoty;
    for(int i = 0; i < snapshot.size(); ++i)
    {
        snapshotx.push_back(snapshot.at(i).first);
        snapshoty.push_back(snapshot.at(i).second);
    }
    std::sort(snapshotx.begin(), snapshotx.end());
    std::sort(snapshoty.begin(),snapshoty.end());

    size_t dataSize = snapshot.size();
    size_t q1Idx, q2Idx, q3Idx; //Assumes y quartile indeces are the same as x
    double quartile1x, quartile2x, quartile3x, IQRx, quartile1y, quartile2y, quartile3y, IQRy;

    // Find quartile indices
    q2Idx = dataSize / 2;
    q1Idx = q2Idx / 2;
    q3Idx = (dataSize % 2 == 0) ? (q2Idx + q1Idx) : (q2Idx + q1Idx + 1);

    // Calculate quartiles
    quartile1x = (q2Idx % 2 == 0) ? (snapshotx[q1Idx - 1] + snapshotx[q1Idx]) / 2.0 : snapshotx[q1Idx];
    quartile2x = (dataSize % 2 == 0) ? (snapshotx[q2Idx - 1] + snapshotx[q2Idx]) / 2.0 : snapshotx[q2Idx];
    quartile3x = (q2Idx % 2 == 0) ? (snapshotx[q3Idx - 1] + snapshotx[q3Idx]) / 2.0 : snapshotx[q3Idx];

    quartile1y = (q2Idx % 2 == 0) ? (snapshoty[q1Idx - 1] + snapshoty[q1Idx]) / 2.0 : snapshoty[q1Idx];
    quartile2y = (dataSize % 2 == 0) ? (snapshoty[q2Idx - 1] + snapshoty[q2Idx]) / 2.0 : snapshoty[q2Idx];
    quartile3y = (q2Idx % 2 == 0) ? (snapshoty[q3Idx - 1] + snapshoty[q3Idx]) / 2.0 : snapshoty[q3Idx];
    IQRx = quartile3x - quartile1x;
    IQRy = quartile3y - quartile1y;

    // Replace outliers with corresponding quartiles
    for (size_t i = 0; i < dataSize; ++i)
    {
        if (snapshotx[i] < quartile1x - (1.5 * IQRx))
        {
            snapshotx[i] = quartile1x;
        }
        else if (snapshotx[i] > quartile3x + (1.5 * IQRx))
        {
            snapshotx[i] = quartile3x;
        }

        if (snapshoty[i] < quartile1y - (1.5 * IQRy))
        {
            snapshoty[i] = quartile1y;
        }
        else if (snapshoty[i] > quartile3y + (1.5 * IQRy))
        {
            snapshoty[i] = quartile3y;
        }
    }

    // Calculate average
    double sumx = std::accumulate(snapshotx.begin(), snapshotx.end(), 0.0);
    double sumy = std::accumulate(snapshoty.begin(), snapshoty.end(), 0.0);
    double avgx = sumx / dataSize;
    double avgy = sumy / dataSize;


    return {avgx,avgy};
}

