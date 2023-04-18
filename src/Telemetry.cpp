#include "Telemetry.h"

Telemetry ::Telemetry(Hardware *devices, double DRIVETRAINWHEELCIRCUMFERENCE)
{
    hw = devices;
    gpsTotalDistance = 0;
    drivetrainEncoderTotalDistance = 0;
    WHEELCIRCUMFERENCE = DRIVETRAINWHEELCIRCUMFERENCE;
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

    // averag encoders
    int numEncoders = hw->driveTrain.size();
    for (int i = 0; i < numEncoders; i++)
    {
        avgEncoderRotations += hw->driveTrain.get_positions().at(i);
    }
    avgEncoderRotations /= numEncoders;

    // calulate displacement
    displacement = avgEncoderRotations * WHEELCIRCUMFERENCE;

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
    int numEncoders = hw->leftWheels.size(); // Assume left and right side have same number of mootors

    double avgLeftRotationalVel = 0;  // Motors on left side of drivetrain
    double avgRightRotationalVel = 0; // Motors on right side of drivetrain

    for (int i = 0; i < numEncoders; i++)
    {
        avgLeftRotationalVel += hw->leftWheels.get_actual_velocities().at(i); // in RPM
        avgRightRotationalVel += hw->rightWheels.get_actual_velocities().at(i);
    }
    avgLeftRotationalVel /= numEncoders;
    avgRightRotationalVel /= numEncoders;

    double angleFromXAxis = getInertiaHeading() * M_PI / 180; // converted to rads for cos() and sin()
    const double WHEELDIAMETER = WHEELCIRCUMFERENCE / M_PI;

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

    for (int i = 0; i < SNAPSHOTSIZE; ++i)
    {
        xSnapshot.push_back(hw->gps.get_status().x * hw->METERTOINCH);
        ySnapshot.push_back(hw->gps.get_status().y * hw->METERTOINCH);
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

    for (int i = 0; i < SNAPSHOTSIZE; ++i)
    {
        // Correct heading to be always be positive counterclockwise from the x axis bc the gps sensor reads clockwise form the positive y axis
        heading = hw->gps.get_heading();
        if (heading > 0)
            heading = 270 - heading;
        else
            heading = -heading - 90;

        snapshot.push_back(heading);
    }

    return snapShotAverage(snapshot);
}

double Telemetry::getInertiaHeading()
{
    std::vector<double> snapshot;
    double heading;

    for (int i = 0; i < SNAPSHOTSIZE; ++i)
    {
        // Correct heading to be always be positive counterclockwise from the x axis bc the inertia sensor reads clockwise form the x axis
        heading = hw->inertial.get_heading();
        if (heading > 0)
            heading = 360 - heading;
        else
            heading = -heading;

        snapshot.push_back(heading);
    }

    return snapShotAverage(snapshot);
}

void Telemetry::setInertiaHeadingToGPS()
{
    hw->inertial.set_heading(getGPSHeading());
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
    size_t q1Idx, q2Idx, q3Idx, q1Idy, q2Idy, q3Idy;
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
