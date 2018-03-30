#include <SFDrive.h>
#include <ctre/Phoenix.h>
#include <Timer.h>
#include <IterativeRobot.h>
#include <AHRS.h>

using namespace frc;

SFDrive::SFDrive(WPI_TalonSRX * lMotor, WPI_TalonSRX * rMotor, IterativeRobot * bot, AHRS * gyro) :
      m_leftMotor(lMotor), m_rightMotor(rMotor), m_robot(bot), m_gyro(gyro)
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
#define ERROR 300 //UNIMPLEMENTED

void SFDrive::PIDDrive(double leftTicks, double rightTicks)
{
   double leftStart = m_leftMotor->GetSelectedSensorPosition(0);
   double rightStart = m_rightMotor->GetSelectedSensorPosition(0);
   double leftSetPoint = m_leftMotor->GetSelectedSensorPosition(0);
   double rightSetPoint = m_rightMotor->GetSelectedSensorPosition(0);
   double lastStepTime = Timer().GetFPGATimestamp();
   while (m_robot->IsAutonomous() && leftSetPoint - leftStart < leftTicks && rightSetPoint - rightStart < rightTicks)
   {
      double currStepTime = Timer().GetFPGATimestamp();
      leftSetPoint += (currStepTime - lastStepTime) * TARGET_REV_SEC;
      rightSetPoint += (currStepTime - lastStepTime) * TARGET_REV_SEC;
      lastStepTime = currStepTime;
      m_leftMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, leftSetPoint);
      m_rightMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, 0 - rightSetPoint);
   }
}

#define TICKS_PER_INCH 217.3
#define ROBOT_DIAMETER 24
#define NUM_CORRECTIONS 2
#define PI 3.14159265

void SFDrive::GyroTurn(double degreesClockwise)
{
   double startAngle = m_gyro->GetAngle();

   for(int i = 0; i < NUM_CORRECTIONS; i++)
   {
      degreesClockwise = m_gyro->GetAngle() - startAngle;
      if(degreesClockwise < 0)
      {
         PIDDrive(0, ((-degreesClockwise) / 360.) * PI * ROBOT_DIAMETER * TICKS_PER_INCH);
      }
      else
      {
         PIDDrive((degreesClockwise / 360.) * PI * ROBOT_DIAMETER * TICKS_PER_INCH, 0);
      }
   }
}

void SFDrive::DriveIntoWall(double timeOut) //I understand the concept of this, but don't know how to implement it and don't have enough time to figure it out
{
   double timeStart = Timer().GetFPGATimestamp();
   while(Timer().GetFPGATimestamp() - timeOut < timeStart)
   {
      ArcadeDrive(1,0); //this is terrifying. Let's never use this.
   }
}
