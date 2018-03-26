#include <ctre/Phoenix.h>
#include <AnalogGyro.h>

#ifndef SRC_SFDRIVE_H_
#define SRC_SFDRIVE_H_

class SFDrive
{
private: //MEMBER VARIABLES
        WPI_TalonSRX * m_leftMotor;
        WPI_TalonSRX * m_rightMotor;
        double m_deadband = 0.08;
        bool isTurning = false;
        AnalogGyro * gyro;

public:
        SFDrive(WPI_TalonSRX * lMotor, WPI_TalonSRX * rMotor, AnalogGyro * _gyro);
        void ArcadeDrive(double xSpeed, double zRotation);
        void GyroTurn(double degrees);
};

#endif
