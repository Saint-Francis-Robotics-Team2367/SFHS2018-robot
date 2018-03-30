#include <SFDrive.h>
#include <ctre/Phoenix.h>
#include <Timer.h>

using namespace frc;

SFDrive::SFDrive(WPI_TalonSRX * lMotor, WPI_TalonSRX * rMotor) :
      m_leftMotor(lMotor), m_rightMotor(rMotor)
{
}

void SFDrive::ArcadeDrive(double xSpeed, double zRotation)
{
   double leftMotorOutput;
   double rightMotorOutput;

   if (fabs(xSpeed) <= m_deadband)
      xSpeed = 0;
   if (fabs(zRotation) <= m_deadband)
      zRotation = 0;

   double maxInput = std::copysign(std::max(std::abs(xSpeed), std::abs(zRotation)), xSpeed);

   if (xSpeed >= 0.0)
   {
      // First quadrant, else second quadrant
      if (zRotation >= 0.0)
      {
         leftMotorOutput = maxInput;
         rightMotorOutput = xSpeed - zRotation;
      }
      else
      {
         leftMotorOutput = xSpeed + zRotation;
         rightMotorOutput = maxInput;
      }
   }
   else
   {
      // Third quadrant, else fourth quadrant
      if (zRotation >= 0.0)
      {
         leftMotorOutput = xSpeed + zRotation;
         rightMotorOutput = maxInput;
      }
      else
      {
         leftMotorOutput = maxInput;
         rightMotorOutput = xSpeed - zRotation;
      }
   }
   m_leftMotor->Set(leftMotorOutput);
   m_rightMotor->Set(-rightMotorOutput);
}

#define TARGET_REV_SEC 8192

void SFDrive::PIDDrive(double _ticks)
{
   double setPoint = 0;
   double lastStepTime = Timer().GetFPGATimestamp();
   while (setPoint < _ticks)
   {
      double currStepTime = Timer().GetFPGATimestamp();
      setPoint += (currStepTime - lastStepTime) * TARGET_REV_SEC;
      lastStepTime = currStepTime;
      m_leftMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, setPoint);
      m_rightMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, 0 - setPoint);
   }
}
