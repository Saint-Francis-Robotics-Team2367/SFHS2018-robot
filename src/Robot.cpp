/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/* ROBOT CHANNEL MAPPINGS
 * 3 5
 * 2 4
 */

#include <iostream>
#include <string>
#include <Drive/DifferentialDrive.h>
#include <Joystick.h>
#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <ctre/Phoenix.h>
#include "SFDrive.h"
#include <Spark.h>
#include <Counter.h>
#include <DigitalInput.h>
#include <DriverStation.h>
#include <Timer.h>
#include <CameraServer.h>
#include <Solenoid.h>
#include <Compressor.h>
#include <BuiltInAccelerometer.h>

#define TRIGGER_DEADZONE 0.1

class Robot : public frc::IterativeRobot
{
   public:
      //Motor channels
      const int joystickNum = 0;
      const int joystickNum2 = 1;
      const int rMotorFrontNum = 5;
      const int rMotorBackNum = 4;
      const int lMotorFrontNum = 3;
      const int lMotorBackNum = 2;
      const int lCubeIntakeNum = 1;
      const int rCubeIntakeNum = 2;
      const int cubeManipAngleNum = 1; //may be incorrect, pls verify

      //Motor tuning constants
      double scale = 1;
      const double TICKS_PER_INCH = 217.3;
      const double TICKS_PER_DEGREE = -3.65;
      double pConstantDrive = 1;
      double iConstantDrive = 0;
      double dConstantDrive = 10;
      double fConstantDrive = 0.1724;
      double pConstantAngle = 1;
      double iConstantAngle = 0;
      double dConstantAngle = 0;
      double angleSpeed = 1;
      double maxDriveMotorCurrent = 30;
      //Misc
      int checkTimeout = 0;
      int timeOut = 100;
      double currentAnglePos;
      double angleIncrement = 2 * TICKS_PER_DEGREE;
      double lastPacket = 0;
      double lastTestPacket = 0;
      double switchPoint = 45 * TICKS_PER_DEGREE;
      double rumbleMultiplier = 1.0/8.0;
      double rumbleDeadzone = 0.5;
      const float gameDataTimeout = 0.1; //amount of time to wait in seconds for game data before defaulting to move forward
      //Starting Data
      std::string position = "RIGHT";
      std::string gameData = "";
      double droppingStart = 0;
      bool upFlag = false;
      double upStart, upTime = 0.5;

   private:
      //Initialize variables
      WPI_TalonSRX * _lMotorFront = new WPI_TalonSRX(lMotorFrontNum);
      WPI_TalonSRX * _lMotorBack = new WPI_TalonSRX(lMotorBackNum);
      WPI_TalonSRX * _rMotorFront = new WPI_TalonSRX(rMotorFrontNum);
      WPI_TalonSRX * _rMotorBack = new WPI_TalonSRX(rMotorBackNum);
      Spark * _lCubeIntake = new Spark(lCubeIntakeNum);
      Spark * _rCubeIntake = new Spark(rCubeIntakeNum);
      WPI_TalonSRX * _cubeManipAngle = new WPI_TalonSRX(cubeManipAngleNum);

      SFDrive *myRobot = new SFDrive(_lMotorFront, _rMotorFront, _lCubeIntake, _rCubeIntake);
      Joystick *stick = new Joystick(joystickNum);
      Joystick *stick2 = new Joystick(joystickNum2);

      BuiltInAccelerometer accelerometer;

      void RobotInit()
      {
         //used to config the motor controllers for QuadEncoders(type of encoder)
         ctre::phoenix::motorcontrol::FeedbackDevice qE = QuadEncoder;
         _lMotorFront->ConfigSelectedFeedbackSensor(qE, 0, checkTimeout);
         _rMotorFront->ConfigSelectedFeedbackSensor(qE, 0, checkTimeout);
         _cubeManipAngle->ConfigSelectedFeedbackSensor(qE, 0, checkTimeout);
         _cubeManipAngle->SetNeutralMode(Brake);
         _cubeManipAngle->GetSensorCollection().SetQuadraturePosition(0, checkTimeout);

         //Set back motors to follower mode
         _lMotorFront->SetSensorPhase(false);
         _lMotorBack->SetSensorPhase(false);

         _rMotorBack->Set(ctre::phoenix::motorcontrol::ControlMode::Follower, rMotorFrontNum);
         _lMotorBack->Set(ctre::phoenix::motorcontrol::ControlMode::Follower, lMotorFrontNum);
         _lMotorFront->SetName("Left Front");
         _rMotorFront->SetName("Right Front");
         _lMotorBack->SetName("Left Back");
         _rMotorBack->SetName("Right Back");
         _cubeManipAngle->SetName("Manipulator Wrist");

         _rMotorFront->SelectProfileSlot(0, 0);
         _rMotorBack->SelectProfileSlot(0, 0);
         _lMotorFront->SelectProfileSlot(0, 0);
         _lMotorBack->SelectProfileSlot(0, 0);
         _cubeManipAngle->SelectProfileSlot(0, 0);

         //Set drive motor max voltage to 30 amps

         _lMotorFront->ConfigContinuousCurrentLimit(maxDriveMotorCurrent - 5, checkTimeout);
         _rMotorFront->ConfigContinuousCurrentLimit(maxDriveMotorCurrent - 5, checkTimeout);
         _lMotorBack->ConfigContinuousCurrentLimit(maxDriveMotorCurrent - 5, checkTimeout);
         _rMotorBack->ConfigContinuousCurrentLimit(maxDriveMotorCurrent - 5, checkTimeout);

         _lMotorFront->ConfigPeakCurrentLimit(maxDriveMotorCurrent, checkTimeout);
         _rMotorFront->ConfigPeakCurrentLimit(maxDriveMotorCurrent, checkTimeout);
         _lMotorBack->ConfigPeakCurrentLimit(maxDriveMotorCurrent, checkTimeout);
         _rMotorBack->ConfigPeakCurrentLimit(maxDriveMotorCurrent, checkTimeout);

         _lMotorFront->ConfigPeakCurrentDuration(0, checkTimeout);
         _rMotorFront->ConfigPeakCurrentDuration(0, checkTimeout);
         _lMotorBack->ConfigPeakCurrentDuration(0, checkTimeout);
         _rMotorBack->ConfigPeakCurrentDuration(0, checkTimeout);

         _lMotorFront->EnableCurrentLimit(true);
         _rMotorFront->EnableCurrentLimit(true);
         _lMotorBack->EnableCurrentLimit(true);
         _rMotorBack->EnableCurrentLimit(true);

         //Shuffleboard
         CameraServer::GetInstance()->StartAutomaticCapture();
         gameData = "";
         position = "RIGHT";
         SmartDashboard::PutBoolean("Allow Field Crossing?", false);
         SmartDashboard::PutString("Starting Position (LEFT, RIGHT, CENTER)", position);
         SmartDashboard::PutNumber("Rumble Multiplier", rumbleMultiplier);
         SmartDashboard::PutNumber("Rumble Deadzone", rumbleDeadzone);

         //list testing block in shuffleboard.
         SmartDashboard::PutNumber("Arc Radius", 46.514);
         SmartDashboard::PutNumber("Arc Angle", 61.73);
         SmartDashboard::PutNumber("maxVel", 55.0);
         SmartDashboard::PutNumber("Shot Travel", 24.0);
         SmartDashboard::PutNumber("auto Timeout", 4.0);
         SmartDashboard::PutNumber("maxAccl", 10000);

         //config acceleraometer
         accelerometer.SetRange(frc::Accelerometer::kRange_8G);
      }

      void RobotPeriodic()
      {
         if (lastPacket + .5 < Timer().GetFPGATimestamp())
         {
            if (!SmartDashboard::ContainsKey("Allow Field Crossing?"))
               SmartDashboard::PutBoolean("Allow Field Crossing?", false);
            if (!SmartDashboard::ContainsKey("Starting Position (LEFT, RIGHT, CENTER)"))
               SmartDashboard::PutString("Starting Position (LEFT, RIGHT, CENTER)", position);
            lastPacket = Timer().GetFPGATimestamp();

            //list testing block in shuffleboard.
            if (!SmartDashboard::ContainsKey("Arc Radius"))
               SmartDashboard::PutNumber("Arc Radius", 46.514);
            if (!SmartDashboard::ContainsKey("Arc Angle"))
               SmartDashboard::PutNumber("Arc Angle", 61.73);
            if (!SmartDashboard::ContainsKey("maxVel"))
               SmartDashboard::PutNumber("maxVel", 4.0 * 12.0);
            if (!SmartDashboard::ContainsKey("Shot Travel"))
               SmartDashboard::PutNumber("Shot Travel", 22.0);
            if (!SmartDashboard::ContainsKey("Shot Start Distance"))
               SmartDashboard::PutNumber("Shot Start Distance", 6.0);
            if (!SmartDashboard::ContainsKey("Shot Time"))
               SmartDashboard::PutNumber("Shot Time", 0.7);
            if (!SmartDashboard::ContainsKey("auto Timeout"))
               SmartDashboard::PutNumber("auto Timeout", 4.0);
         }
      }

      void TeleopInit()
      {
         DriverStation::ReportError("TeleopInit Started");
         //Set encoder positions to 0
         ConfigPIDS();
         myRobot->ArcadeDrive(0.0, 0.0);
         currentAnglePos = _cubeManipAngle->GetSelectedSensorPosition(0);
         DriverStation::ReportError("TeleopInit Completed");

         //list testing block in shuffleboard.
         SmartDashboard::PutNumber("Arc Radius", 46.514);
         SmartDashboard::PutNumber("Arc Angle", 61.73);
         SmartDashboard::PutNumber("maxVel", 55.0);
         SmartDashboard::PutNumber("Shot Travel", 24.0);
         SmartDashboard::PutNumber("Shot Start Distance", 8.0);
         SmartDashboard::PutNumber("Shot Time", 0.7);
         SmartDashboard::PutNumber("auto Timeout", 4.0);
         SmartDashboard::PutNumber("maxAccl", 10000);
         DriverStation::ReportError("TestInit Completed");
      }

      void TeleopPeriodic()
      {
         myRobot->ArcadeDrive(scale * stick->GetRawAxis(1), -(stick->GetRawAxis(4) > 0 ? 1 : -1) * stick->GetRawAxis(4) * stick->GetRawAxis(4));

         myRobot->setAccel(SmartDashboard::GetNumber("maxAccl", 8000));
         SmartDashboard::PutNumber("Left Encoder", _lMotorFront->GetSelectedSensorPosition(0));
         SmartDashboard::PutNumber("Right Encoder", _rMotorFront->GetSelectedSensorPosition(0));

         if (stick->GetRawButton(2)) //b
         {
            this->_lCubeIntake->Set(1);
            this->_rCubeIntake->Set(1);
         }
         else
         {
            if (stick->GetRawAxis(3) > TRIGGER_DEADZONE) //right trigger
            {
               this->_lCubeIntake->Set(-stick->GetRawAxis(3));
            }
            else
            {
               this->_lCubeIntake->Set(0);
            }
            if (stick->GetRawAxis(2) > TRIGGER_DEADZONE) //left trigger
            {
               this->_rCubeIntake->Set(-stick->GetRawAxis(2));
            }
            else
            {
               this->_rCubeIntake->Set(0);
            }
         }
         if (stick2->GetRawButtonReleased(1)) // a button
         {
            currentAnglePos = TICKS_PER_DEGREE * 90;
         }
         else if (stick2->GetRawButtonReleased(4)) // y button
         {
            currentAnglePos = TICKS_PER_DEGREE * 0;
         }
         else if (stick2->GetRawButtonReleased(3)) // x button
         {
            currentAnglePos = switchPoint;
         }

         if (stick->GetRawButton(6)) // left bumper
         {
            upFlag = false;
            _cubeManipAngle->ConfigPeakCurrentLimit(5, checkTimeout);
            _cubeManipAngle->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -1);
         }
         else if (!upFlag)
         {
            _cubeManipAngle->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
         }
         if (stick->GetRawButton(5)) //right bumper
         {
            upFlag = true;
            _cubeManipAngle->ConfigPeakCurrentLimit(15, checkTimeout);
            upStart = Timer().GetFPGATimestamp() + upTime;
         }
         if (upFlag == true && upStart > Timer().GetFPGATimestamp())
         {
            _cubeManipAngle->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.8);
         }
         else
         {
            upFlag = false;
         }

         double acceleration = std::pow(accelerometer.GetX() * accelerometer.GetX() + accelerometer.GetY() * accelerometer.GetY(), 0.5);
         rumbleMultiplier = SmartDashboard::GetNumber("Rumble Multiplier", rumbleMultiplier);
         rumbleDeadzone = SmartDashboard::GetNumber("Rumble Deadzone", rumbleDeadzone);
         if(acceleration < rumbleDeadzone)
            acceleration=0;

         stick->SetRumble(GenericHID::RumbleType::kLeftRumble, acceleration * rumbleMultiplier);
         stick->SetRumble(GenericHID::RumbleType::kRightRumble, acceleration * rumbleMultiplier);



         /*
          if(stick->GetRawButton(6))
          {
          _cubeManipAngle->ConfigPeakCurrentLimit (5, checkTimeout);
          _cubeManipAngle->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, .8);
          }
          else if(stick->GetRawButton(5)) // right bumper
          {

          _cubeManipAngle->ConfigPeakCurrentLimit (15, checkTimeout);
          _cubeManipAngle->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -1);
          }
          else if(!(stick->GetRawButton(5) || stick2->GetRawButton(6)))
          _cubeManipAngle->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
          */

      }

      void AutonomousInit()
      {
         DriverStation::ReportError("AutonInit Started");

         /* Side info
          * move 14 feet up
          * cross?
          * got 6 feet, turn, go 19 feet, turn, go 6 feet, turn, go 4 feet
          * otherwise
          * turn and go 4 feet
          */

         position = SmartDashboard::GetString("Starting Position (LEFT, RIGHT, CENTER)", "RIGHT");

         float radius, maxVel, shotTravel, timeout;
         radius = SmartDashboard::GetNumber("Arc Radius", 46.514);
         maxVel = SmartDashboard::GetNumber("maxVel", 55);
         shotTravel = SmartDashboard::GetNumber("Shot Travel", 24.0);
         timeout = SmartDashboard::GetNumber("auto Timeout", 4.0);

         double autoStart = Timer().GetFPGATimestamp();
         while (gameData == "" && Timer().GetFPGATimestamp() < autoStart + gameDataTimeout)
         {
            gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage().substr(0, 1);
         }


         if (gameData == "")
         {
            DriverStation::ReportError("No game data, defaulting to forward.");
            myRobot->PIDDrive(60, maxVel, timeout, true);
            return;
         }

         if (position != "LEFT" && position != "CENTER" && position != "RIGHT")
         {
            DriverStation::ReportError("Error setting position! Defaulting to LEFT");
            position = "LEFT";
         }

         DriverStation::ReportError("AUTON START. Starting Position: " + position + " Switch side: " + gameData);

         int errorCode = 0;

         if (position == "LEFT")
         {
            errorCode += myRobot->PIDDrive(92, maxVel, timeout, true);
            errorCode += myRobot->PIDTurn(85, radius, maxVel, timeout, true);
            if (gameData == "L")
            {
               errorCode += myRobot->PIDShoot(40, 34, 3, maxVel, timeout);
            }
         }
         else if (position == "CENTER")
         {
            if (gameData == "L")
            {
               myRobot->PIDTurn(61.73 * -1.0f, 46.514, maxVel, timeout, true);
               myRobot->PIDTurn(61.73, 46.514, maxVel, timeout, true);
               myRobot->PIDShoot(shotTravel, 12, 0.7, maxVel, timeout);
            }
            else if (gameData == "R")
            {
               myRobot->PIDTurn(61.73, 46.514, maxVel, timeout, true);
               myRobot->PIDTurn(61.73 * -1.0f, 46.514, maxVel, timeout, true);
               myRobot->PIDShoot(shotTravel, 12, 0.7, maxVel, timeout);
            }
         }
         else if (position == "RIGHT")
         {
            errorCode += myRobot->PIDDrive(92, maxVel, timeout, true);
            errorCode += myRobot->PIDTurn(-85, radius, maxVel, timeout, true);
            if (gameData == "R")
            {
               errorCode += myRobot->PIDShoot(40, 34, 3, maxVel, timeout);
            }
         }
         DriverStation::ReportError("Auton completed with " + std::to_string(errorCode) + " error(s).");
      }

      void AutonomousPeriodic()
      {

      }

      void TestInit()
      {
         DriverStation::ReportError("TestInit Started");
         //Set encoder positions to 0
         ConfigPIDS();
         myRobot->ArcadeDrive(0.0, 0.0);
         currentAnglePos = _cubeManipAngle->GetSelectedSensorPosition(0);
         DriverStation::ReportError("TestInit Completed");
      }

      void TestPeriodic()
      {
         myRobot->ArcadeDrive(scale * stick->GetRawAxis(1), -(stick->GetRawAxis(4) > 0 ? 1 : -1) * stick->GetRawAxis(4) * stick->GetRawAxis(4));
         myRobot->setAccel(SmartDashboard::GetNumber("maxAccl", 8000));
         SmartDashboard::PutNumber("Left Encoder", _lMotorFront->GetSelectedSensorPosition(0));
         SmartDashboard::PutNumber("Right Encoder", _rMotorFront->GetSelectedSensorPosition(0));

         if (stick->GetRawButton(7))
         {
            float radius, angle, maxVel, shotTravel, timeout;
            radius = SmartDashboard::GetNumber("Arc Radius", 46.514);
            angle = SmartDashboard::GetNumber("Arc Angle", 61.73);
            maxVel = SmartDashboard::GetNumber("maxVel", 55);
            shotTravel = SmartDashboard::GetNumber("Shot Travel", 24.0);
            timeout = SmartDashboard::GetNumber("auto Timeout", 4.0);

            int errCnt = 0;
            errCnt += myRobot->PIDTurn(angle * -1.0f, radius, maxVel, timeout, true);
            errCnt += myRobot->PIDTurn(angle, radius, maxVel, timeout, true);
            errCnt += myRobot->PIDDrive(shotTravel, maxVel, timeout, true);
            errCnt += myRobot->PIDDrive(-27, maxVel, timeout, true);
            errCnt += myRobot->PIDTurn(85, 0, maxVel, timeout, true);
            errCnt += myRobot->PIDDrive(30, maxVel, timeout, true);
            errCnt += myRobot->PIDDrive(-57, maxVel, timeout, true);
            errCnt += myRobot->PIDTurn(-85, 0, maxVel, timeout, true);
            errCnt += myRobot->PIDDrive(13, maxVel, timeout, true);

            if (errCnt)
               DriverStation::ReportError("PID Timeout" + errCnt);
         }
         if (stick->GetRawButton(8))
         {
            float maxVel, shotTravel, timeout;
            maxVel = SmartDashboard::GetNumber("maxVel", 80.0);
            shotTravel = SmartDashboard::GetNumber("Shot Travel", 24.0);
            timeout = SmartDashboard::GetNumber("auto Timeout", 4.0);

            int errCnt = 0;
            errCnt += myRobot->PIDDrive(shotTravel, maxVel, timeout, true);
            if (errCnt)
               DriverStation::ReportError("PID Timeout" + errCnt);
         }

         if (stick->GetRawButtonReleased(1)) // a button
         {
            currentAnglePos = TICKS_PER_DEGREE * 90;
         }
         else if (stick->GetRawButtonReleased(4)) // y button
         {
            currentAnglePos = TICKS_PER_DEGREE * 0;
         }
         else if (stick->GetRawButtonReleased(3)) // x button
         {
            currentAnglePos = switchPoint;
         }
         else if (stick->GetRawButton(6)) // left bumper
         {
            _cubeManipAngle->Set(1);
         }
         else if (stick->GetRawButton(5)) // right bumper
         {
            _cubeManipAngle->Set(-1);
         }
         else if (!(stick->GetRawButton(5) || stick2->GetRawButton(6)))
         {
            _cubeManipAngle->Set(0);
         }

      }

      void ConfigPIDS()
      {
         DriverStation::ReportError("PID Config Started");

         _rMotorBack->SetNeutralMode(Brake);
         _lMotorBack->SetNeutralMode(Brake);

         _rMotorFront->GetSensorCollection().SetQuadraturePosition(0, checkTimeout);
         _rMotorBack->GetSensorCollection().SetQuadraturePosition(0, checkTimeout);
         _lMotorFront->GetSensorCollection().SetQuadraturePosition(0, checkTimeout);
         _lMotorBack->GetSensorCollection().SetQuadraturePosition(0, checkTimeout);

         _lMotorFront->Config_kP(0, pConstantDrive, checkTimeout);
         _lMotorFront->Config_kI(0, iConstantDrive, checkTimeout);
         _lMotorFront->Config_kD(0, dConstantDrive, checkTimeout);
         _lMotorFront->Config_kF(0, fConstantDrive, checkTimeout);

         _lMotorBack->Config_kP(0, pConstantDrive, checkTimeout);
         _lMotorBack->Config_kI(0, iConstantDrive, checkTimeout);
         _lMotorBack->Config_kD(0, dConstantDrive, checkTimeout);
         _lMotorBack->Config_kF(0, fConstantDrive, checkTimeout);

         _rMotorFront->Config_kP(0, pConstantDrive, checkTimeout);
         _rMotorFront->Config_kI(0, iConstantDrive, checkTimeout);
         _rMotorFront->Config_kD(0, dConstantDrive, checkTimeout);
         _rMotorFront->Config_kF(0, fConstantDrive, checkTimeout);

         _rMotorBack->Config_kP(0, pConstantDrive, checkTimeout);
         _rMotorBack->Config_kI(0, iConstantDrive, checkTimeout);
         _rMotorBack->Config_kD(0, dConstantDrive, checkTimeout);
         _rMotorBack->Config_kF(0, fConstantDrive, checkTimeout);

         _cubeManipAngle->Config_kP(0, pConstantAngle, checkTimeout);
         _cubeManipAngle->Config_kI(0, iConstantAngle, checkTimeout);
         _cubeManipAngle->Config_kD(0, dConstantAngle, checkTimeout);
         _cubeManipAngle->Config_kF(0, fConstantDrive, checkTimeout);

         DriverStation::ReportError("PID Config Completed");
      }

      void DisabledInit()
      {
         stick->SetRumble(GenericHID::RumbleType::kLeftRumble, 0);
         stick->SetRumble(GenericHID::RumbleType::kRightRumble, 0);
      }

      void DisabledPeriodic()
      {
         _lMotorFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
         _rMotorFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
      }
}
;

START_ROBOT_CLASS(Robot)
