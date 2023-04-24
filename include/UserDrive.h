#include "Drive.h"

class UserDrive : public Drive{
public:
    UserDrive(Hardware* hardware, RobotConfig* robotConfig, Telemetry* telemetry);
    int getFlywheelSpeed();
    void drive();

private:
    
    const int deadzone = 5;

    double flywheelVoltage = 8;

    int mirrorDrive = 1;

    /// @brief  Controls drivetrain based on controller joysticks. Up/down on the left joystick is forward/backward.
    //          Left/right on the right joystick is turning.
    void driveTrainControls();

    /// @brief  Runs intake  while thee L1 conroller bumper is held and reverse intake  while the X button is held.
    void intakeControls();

    /// @brief  Spins up flywheel while R1 controller bumper is held.
    void flywheelControls();

    /// @brief  Flicks disk once when the R2 controller bumper is pressed. Flicks multiple times if held.
    void flickDiskControls();

    /// @brief  Lauches the expansion when the A buutton is pressed.
    void expandControls();

    /// @brief Toggles mirror drive variable
    void mirrorDriveToggle();
};