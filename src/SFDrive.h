#include <ctre/Phoenix.h>
#include <IterativeRobot.h>
#include <AHRS.h>

#ifndef SRC_SFDRIVE_H_
#define SRC_SFDRIVE_H_

class SFDrive
{
private: //MEMBER VARIABLES
        WPI_TalonSRX * m_leftMotor;
        WPI_TalonSRX * m_rightMotor;
        IterativeRobot * m_robot;
        AHRS * m_gyro;
        double m_deadband = 0.08;

public:
        SFDrive(WPI_TalonSRX * lMotor, WPI_TalonSRX * rMotor, IterativeRobot * bot, AHRS * gyro);
        void ArcadeDrive(double xSpeed, double zRotation);
        void PIDDrive(double leftTicks, double rightTicks);
        void GyroTurn(double degreesClockwise);
        void DriveIntoWall(double timeOut);
};

#endif
