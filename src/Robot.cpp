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
#include <MotionProfileExample.h>
#include <MotionProfile.h>

#define TRIGGER_DEADZONE 0.1

class Robot : public frc::IterativeRobot
{
   public:
      //Motor channels
      const int joystickNum = 0;
      const int rMotorFrontNum = 5;
      const int rMotorBackNum = 4;
      const int lMotorFrontNum = 3;
      const int lMotorBackNum = 2;
      const int lCubeIntakeNum = 1;
      const int rCubeIntakeNum = 2;
      const int cubeManipAngleOpenNum = 1;
      const int cubeManipAngleCloseNum = 0;

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
      double angleIncrement = 2 * TICKS_PER_DEGREE;
      double lastPacket = 0;
      double lastTestPacket = 0;
      double switchPoint = 45 * TICKS_PER_DEGREE; //TODO
      //Starting Data
      std::string position = "LEFT";
      std::string gameData = "";
      std::string mode = "BASIC";
      bool allowFieldCrossing = false;
      bool autonHasRun = false;
      double matchStart;
      bool isDropping;
      double droppingStart = 0;

   private:
      //Initialize variables
      WPI_TalonSRX * _lMotorFront = new WPI_TalonSRX(lMotorFrontNum);
      WPI_TalonSRX * _lMotorBack = new WPI_TalonSRX(lMotorBackNum);
      WPI_TalonSRX * _rMotorFront = new WPI_TalonSRX(rMotorFrontNum);
      WPI_TalonSRX * _rMotorBack = new WPI_TalonSRX(rMotorBackNum);
      Spark * _lCubeIntake = new Spark(lCubeIntakeNum);
      Spark * _rCubeIntake = new Spark(rCubeIntakeNum);
      Solenoid * _cubeAngleManipOpen = new Solenoid(cubeManipAngleOpenNum);
      Solenoid * _cubeAngleManipClose = new Solenoid(cubeManipAngleCloseNum);
      MotionProfileExample * lMotionProfile = new MotionProfileExample(*_lMotorFront);
      MotionProfileExample * rMotionProfile = new MotionProfileExample(*_rMotorFront);
      Compressor * compressor = new Compressor(0);

      SFDrive *myRobot = new SFDrive(_lMotorFront, _rMotorFront);
      Joystick *stick = new Joystick(joystickNum);

      void RobotInit()
      {
         //used to config the motor controllers for QuadEncoders(type of encoder)
         ctre::phoenix::motorcontrol::FeedbackDevice qE = QuadEncoder;
         _lMotorFront->ConfigSelectedFeedbackSensor(qE, 0, checkTimeout);
         _rMotorFront->ConfigSelectedFeedbackSensor(qE, 0, checkTimeout);

         lMotionProfile->phase = true;

         //Set back motors to follower mode
         _lMotorFront->SetSensorPhase(false);
         _lMotorBack->SetSensorPhase(false);

         _rMotorBack->Set(ctre::phoenix::motorcontrol::ControlMode::Follower, rMotorFrontNum);
         _lMotorBack->Set(ctre::phoenix::motorcontrol::ControlMode::Follower, lMotorFrontNum);
         _lMotorFront->SetName("Left Front");
         _rMotorFront->SetName("Right Front");
         _lMotorBack->SetName("Left Back");
         _rMotorBack->SetName("Right Back");

         _rMotorFront->SelectProfileSlot(0, 0);
         _rMotorBack->SelectProfileSlot(0, 0);
         _lMotorFront->SelectProfileSlot(0, 0);
         _lMotorBack->SelectProfileSlot(0, 0);

         //Set drive motor max voltage to 30 amps
         _lMotorFront->ConfigPeakCurrentLimit(maxDriveMotorCurrent, checkTimeout);
         _rMotorFront->ConfigPeakCurrentLimit(maxDriveMotorCurrent, checkTimeout);
         _lMotorBack->ConfigPeakCurrentLimit(maxDriveMotorCurrent, checkTimeout);
         _rMotorBack->ConfigPeakCurrentLimit(maxDriveMotorCurrent, checkTimeout);

         //Shuffleboard
         CameraServer::GetInstance()->StartAutomaticCapture();
         SmartDashboard::PutString("Mode (NOTHING, BASIC, INTERMEDIATE, ADVANCED, EMERGENCY)", mode);
         SmartDashboard::PutBoolean("Allow Field Crossing?", false);
         SmartDashboard::PutString("Starting Position (LEFT, RIGHT, CENTER)", position);

         //Pneumatics
         compressor->Enabled();
         _cubeAngleManipOpen->Set(true);
         _cubeAngleManipClose->Set(false);
      }

      void RobotPeriodic()
      {
         if (gameData == "")
            gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage().substr(0, 1);
         if (lastPacket + .5 < Timer().GetFPGATimestamp())
         {
            if (!SmartDashboard::ContainsKey("Mode (NOTHING, BASIC, INTERMEDIATE, ADVANCED, EMERGENCY)"))
               SmartDashboard::PutString("Mode (NOTHING, BASIC, INTERMEDIATE, ADVANCED, EMERGENCY)", mode);
            if (!SmartDashboard::ContainsKey("Allow Field Crossing?"))
               SmartDashboard::PutBoolean("Allow Field Crossing?", false);
            if (!SmartDashboard::ContainsKey("Starting Position (LEFT, RIGHT, CENTER)"))
               SmartDashboard::PutString("Starting Position (LEFT, RIGHT, CENTER)", position);
            lastPacket = Timer().GetFPGATimestamp();
         }

      }

      void TeleopInit()
      {
         DriverStation::ReportError("TeleopInit Started");
         //Set encoder positions to 0
         ConfigPIDS();
         myRobot->ArcadeDrive(0.0, 0.0);
         DriverStation::ReportError("TeleopInit Completed");
      }

      void TeleopPeriodic()
      {
         myRobot->ArcadeDrive(scale * stick->GetRawAxis(1), -(stick->GetRawAxis(4) > 0 ? 1 : -1) * stick->GetRawAxis(4) * stick->GetRawAxis(4));

         if (stick->GetRawAxis(3) > TRIGGER_DEADZONE) //right trigger
         {
            this->_lCubeIntake->Set(-stick->GetRawAxis(3));
         }
         if (stick->GetRawAxis(2) > TRIGGER_DEADZONE) //left trigger
         {
            this->_rCubeIntake->Set(-stick->GetRawAxis(2));
         }
         if (stick->GetRawButton(5))
         {
            _cubeAngleManipOpen->Set(true);
            _cubeAngleManipClose->Set(false);
         }
         else if (stick->GetRawButton(6))
         {
            _cubeAngleManipOpen->Set(false);
            _cubeAngleManipClose->Set(true);
         }
         if (stick->GetRawButton(2)) //b
         {
            this->_lCubeIntake->Set(1);
            this->_rCubeIntake->Set(1);
         }
         if (!(stick->GetRawAxis(3) > TRIGGER_DEADZONE || stick->GetRawAxis(2) > TRIGGER_DEADZONE || stick->GetRawButton(2)))
         {
            this->_lCubeIntake->Set(0);
            this->_rCubeIntake->Set(0);
         }
      }

      void AutonomousInit()
      {
         DriverStation::ReportError("AutonInit Started");

         position = SmartDashboard::GetString("Starting Position (LEFT, RIGHT, CENTER)", "LEFT");
         mode = SmartDashboard::GetString("Mode (NOTHING, BASIC, INTERMEDIATE, ADVANCED, EMERGENCY)", "BASIC");
         allowFieldCrossing = SmartDashboard::GetBoolean("Allow Field Crossing?", false);

         if (mode != "NOTHING" && mode != "BASIC" && mode != "INTERMEDIATE" && mode != "ADVANCED" && mode != "EMERGENCY")
         {
            DriverStation::ReportError("Error setting auton mode! Defaulting to BASIC");
            mode = "BASIC";
         }

         if (position != "LEFT" && position != "CENTER" && position != "RIGHT")
         {
            DriverStation::ReportError("Error setting position! Defaulting to LEFT");
            position = "LEFT";
         }

         if (mode != "NOTHING" && mode != "EMERGENCY")
         {
            ConfigPIDS();
            _lMotorFront->Set(ctre::phoenix::motorcontrol::ControlMode::MotionProfile, 1);
            _rMotorFront->Set(ctre::phoenix::motorcontrol::ControlMode::MotionProfile, 1);
         }
         else
            autonHasRun = true;
         matchStart = Timer().GetFPGATimestamp();
         DriverStation::ReportError("AutonInit Completed. Mode: " + mode + " Starting Position: " + position + " Cross field? " + (allowFieldCrossing ? "True" : "False") + " Switch side: " + gameData);
      }

      void AutonomousPeriodic()
      {
         _cubeAngleManipOpen->Set(true);
         _cubeAngleManipClose->Set(false);
         if (!autonHasRun)
         {
            if (gameData == "")
               return;
            autonHasRun = true;
            if (mode == "BASIC" || mode == "INTERMEDIATE" || mode == "ADVANCED")
            {
               if (position == "LEFT")
               {
                  if (gameData == "L")
                  {
                     lMotionProfile->startFilling(motionProfile_left_left_left, count_left_left_left);
                     rMotionProfile->startFilling(motionProfile_left_left_right, count_left_left_right);
                  }
                  else if (gameData == "R")
                  {
                     if (allowFieldCrossing)
                     {
                        lMotionProfile->startFilling(motionProfile_left_right_left, count_left_right_left);
                        rMotionProfile->startFilling(motionProfile_left_right_right, count_left_right_right);
                     }
                     else
                     {
                        lMotionProfile->startFilling(motionProfile_left_left_left, count_left_left_left);
                        rMotionProfile->startFilling(motionProfile_left_left_right, count_left_left_right);
                     }
                  }
               }
               else if (position == "CENTER")
               {
                  if (gameData == "L")
                  {
                     lMotionProfile->startFilling(motionProfile_center_left_left, count_center_left_left);
                     rMotionProfile->startFilling(motionProfile_center_left_right, count_center_left_right);
                  }
                  else if (gameData == "R")
                  {
                     lMotionProfile->startFilling(motionProfile_center_right_left, count_center_right_left);
                     rMotionProfile->startFilling(motionProfile_center_right_right, count_center_right_right);
                  }
               }
               else if (position == "RIGHT")
               {
                  if (gameData == "L")
                  {
                     if (allowFieldCrossing)
                     {
                        lMotionProfile->startFilling(motionProfile_right_left_left, count_right_left_left);
                        rMotionProfile->startFilling(motionProfile_right_left_right, count_right_left_right);
                     }
                     else
                     {
                        lMotionProfile->startFilling(motionProfile_right_right_left, count_right_right_left);
                        rMotionProfile->startFilling(motionProfile_right_right_right, count_right_right_right);
                     }
                  }
                  else if (gameData == "R")
                  {
                     lMotionProfile->startFilling(motionProfile_right_right_left, count_right_right_left);
                     rMotionProfile->startFilling(motionProfile_right_right_right, count_right_right_right);
                  }
               }
            }
            lMotionProfile->start();
            rMotionProfile->start();
         }
         if (mode == "EMERGENCY")
         {
            if (Timer().GetFPGATimestamp() - matchStart < 4)
               myRobot->ArcadeDrive(-0.5, 0);
            else
               myRobot->ArcadeDrive(0, 0);
         }
         if (mode == "INTERMEDIATE" || mode == "ADVANCED")
         {
            if (!((position == "LEFT" && gameData == "R" && allowFieldCrossing == false) || (position == "RIGHT" && gameData == "L" && allowFieldCrossing == false)))
            {
               if ((!isDropping && Timer().GetFPGATimestamp() - matchStart > 4 && _lMotorFront->GetMotionProfileTopLevelBufferCount() + _rMotorFront->GetMotionProfileTopLevelBufferCount() == 0))
               {
                  isDropping = true;
                  droppingStart = Timer().GetFPGATimestamp();
                  _lMotorFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -0.5);
                  _rMotorFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.5);
               }
            }
         }
         if (isDropping)
         {
            if (Timer().GetFPGATimestamp() - droppingStart > 5)
            {
               _lMotorFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
               _rMotorFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
               _lCubeIntake->Set(1);
               _rCubeIntake->Set(1);
               isDropping = false;
            }
         }
      }

      void TestInit()
      {
         DriverStation::ReportError("TestInit Started");
         ConfigPIDS();
         /*Put PID values in ShuffleBoard
          SmartDashboard::PutNumber ("P Drive", pConstantDrive);
          SmartDashboard::PutNumber ("I Drive", iConstantDrive);
          SmartDashboard::PutNumber ("D Drive", dConstantDrive);
          SmartDashboard::PutNumber ("P Angle", pConstantAngle);
          SmartDashboard::PutNumber ("I Angle", iConstantAngle);
          SmartDashboard::PutNumber ("D Angle", dConstantAngle);
          SmartDashboard::PutNumber ("Setpoint Drive", 0);
          SmartDashboard::PutNumber ("Setpoint Angle", MAX_ANGLE_TICKS);
          SmartDashboard::PutNumber ("Current Position - Right", 0);
          SmartDashboard::PutNumber ("Current Position - Left", 0);
          SmartDashboard::PutNumber ("Current Position - Angle", MAX_ANGLE_TICKS);
          */
         DriverStation::ReportError("TestInit Completed");
      }

      void TestPeriodic()
      {
         /*
          if (lastTestPacket + 0.5 < Timer ().GetFPGATimestamp ()) //Update PID and setpoint values from shuffleboard
          {
          //Every 100 packets (2 seconds), update P, I, D values
          pConstantDrive = SmartDashboard::GetNumber ("P Drive", pConstantDrive);
          iConstantDrive = SmartDashboard::GetNumber ("I Drive", iConstantDrive);
          dConstantDrive = SmartDashboard::GetNumber ("D Drive", dConstantDrive);
          pConstantAngle = SmartDashboard::GetNumber ("P Angle", pConstantAngle);
          iConstantAngle = SmartDashboard::GetNumber ("I Angle", iConstantAngle);
          dConstantAngle = SmartDashboard::GetNumber ("D Angle", dConstantAngle);
          setPointDrive = SmartDashboard::GetNumber ("Setpoint Drive", setPointDrive);
          setPointAngle = SmartDashboard::GetNumber ("Setpoint Angle", setPointAngle);
          SmartDashboard::PutNumber ("Current Position - Right", _rMotorFront->GetSensorCollection ().GetQuadraturePosition ());
          SmartDashboard::PutNumber ("Current Position - Left", _lMotorFront->GetSensorCollection ().GetQuadraturePosition ());
          SmartDashboard::PutNumber ("Current Position - Angle", _cubeManipAngle->GetSensorCollection ().GetQuadraturePosition ());
          _lMotorFront->Config_kP (0, pConstantDrive, checkTimeout);
          _lMotorFront->Config_kI (0, iConstantDrive, checkTimeout);
          _lMotorFront->Config_kD (0, dConstantDrive, checkTimeout);
          _lMotorBack->Config_kP (0, pConstantDrive, checkTimeout);
          _lMotorBack->Config_kI (0, iConstantDrive, checkTimeout);
          _lMotorBack->Config_kD (0, dConstantDrive, checkTimeout);
          _rMotorFront->Config_kP (0, pConstantDrive, checkTimeout);
          _rMotorFront->Config_kI (0, iConstantDrive, checkTimeout);
          _rMotorFront->Config_kD (0, dConstantDrive, checkTimeout);
          _rMotorBack->Config_kP (0, pConstantDrive, checkTimeout);
          _rMotorBack->Config_kI (0, iConstantDrive, checkTimeout);
          _rMotorBack->Config_kD (0, dConstantDrive, checkTimeout);
          _cubeManipAngle->Config_kP (0, pConstantAngle, checkTimeout);
          _cubeManipAngle->Config_kI (0, iConstantAngle, checkTimeout);
          _cubeManipAngle->Config_kD (0, dConstantAngle, checkTimeout);
          lastTestPacket = Timer ().GetFPGATimestamp ();
          }
          */
         myRobot->ArcadeDrive(1, 0);
         //_rMotorFront->Set(ctre::phoenix::motorcontrol::ControlMode::Position, 25000);
         //_lMotorFront->Set(ctre::phoenix::motorcontrol::ControlMode::Position, -25000);
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

         DriverStation::ReportError("PID Config Completed");
      }

      void DisabledInit()
      {
         lMotionProfile->reset();
         rMotionProfile->reset();
      }

      void DisabledPeriodic()
      {
         _lMotorFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
         _rMotorFront->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
      }
};

START_ROBOT_CLASS(Robot)
