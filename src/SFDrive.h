#include <ctre/Phoenix.h>

#ifndef SRC_SFDRIVE_H_
#define SRC_SFDRIVE_H_

class SFDrive
{
private: //MEMBER VARIABLES
        WPI_TalonSRX * m_leftMotor;
        WPI_TalonSRX * m_rightMotor;
        double m_deadband = 0.08;
        double m_lastPIDTime = 0;
        int m_PIDStepTime = 10 ^ 8;
        double m_PIDStepSize = 500;
        const float m_ticksPerRev = 1024 * 4;
        const float m_wheelCircumference = 6 * 3.14;
        const float m_wheelTrack = 24;
        float m_currVelocity = 0;
        const float m_maxAccl = 8000;
        const float m_minDecelVel = 3000;
        const float m_P = 1;
        const float m_I = 0;
        const float m_D = 10;
        const float m_canTimeout = 0;

public:
        SFDrive(WPI_TalonSRX * lMotor, WPI_TalonSRX * rMotor);
        void ArcadeDrive(double xSpeed, double zRotation);
        bool PIDShoot(float moveInches, float shootStartDist, float shootTime, float maxVel, float timeout = 4);
        bool PIDDrive(float inches, float maxVel, float timeout = 4, bool ZeroVelocityAtEnd = true);
        bool PIDTurn(float degreesClockwise, float radius, float maxVel, float timeout = 4, bool ZeroVelocityAtEnd = true);
        void initPID();
        void disableP();
        void enableP();

        void setAccel(float);
};

#endif
