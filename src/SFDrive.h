#include <ctre/Phoenix.h>
#include <IterativeRobot.h>

#ifndef SRC_SFDRIVE_H_
#define SRC_SFDRIVE_H_

class SFDrive
{
private: //MEMBER VARIABLES
        WPI_TalonSRX * m_leftMotor;
        WPI_TalonSRX * m_rightMotor;
        IterativeRobot * m_robot;
        double m_deadband = 0.08;
        double m_lastPIDTime = 0;
        int m_PIDStepTime = 10 ^ 8;
        double m_PIDStepSize = 500;

public:
        SFDrive(WPI_TalonSRX * lMotor, WPI_TalonSRX * rMotor, IterativeRobot * bot);
        void ArcadeDrive(double xSpeed, double zRotation);
        void PIDDrive(double leftTicks, double rightTicks);
        void GyroTurn(double degreesClockwise);
};

#endif
