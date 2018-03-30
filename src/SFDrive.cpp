#include <SFDrive.h>
#include <ctre/Phoenix.h>
#include <Timer.h>
#include <IterativeRobot.h>

using namespace frc;

SFDrive::SFDrive(WPI_TalonSRX * lMotor, WPI_TalonSRX * rMotor, IterativeRobot * bot) :
      m_leftMotor(lMotor), m_rightMotor(rMotor), m_robot(bot)
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

void SFDrive::PIDDrive(double leftTicks, double rightTicks)
{
   double leftSetPoint = 0;
   double rightSetPoint = 0;
   double lastStepTime = Timer().GetFPGATimestamp();
   while (m_robot->IsAutonomous() && leftSetPoint < leftTicks && rightSetPoint < rightTicks)
   {
      double currStepTime = Timer().GetFPGATimestamp();
      leftSetPoint += (currStepTime - lastStepTime) * TARGET_REV_SEC;
      rightSetPoint += (currStepTime - lastStepTime) * TARGET_REV_SEC;
      lastStepTime = currStepTime;
      m_leftMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, leftSetPoint);
      m_rightMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, 0 - rightSetPoint);
   }
}

void SFDrive::GyroTurn(double degreesClockwise)
{
   return;
}
