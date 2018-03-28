#include <ctre/Phoenix.h>
#include <AnalogGyro.h>
#include <AHRS.h>

#ifndef SRC_SFDRIVE_H_
#define SRC_SFDRIVE_H_

class SFDrive
{
   private:
      //MEMBER VARIABLES
      WPI_TalonSRX * m_leftMotor;
      WPI_TalonSRX * m_rightMotor;
      double m_deadband = 0.08;
      bool isTurning = false;
      double turnSpeed = 0.2;
      double allowedTurningError = 1;
      AHRS * gyro;

   public:
      SFDrive(WPI_TalonSRX * lMotor, WPI_TalonSRX * rMotor, AHRS * _gyro);
      void ArcadeDrive(double xSpeed, double zRotation);
      void GyroTurn(double degrees);
};

#endif
