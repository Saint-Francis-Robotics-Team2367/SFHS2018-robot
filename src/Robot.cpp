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
#include "AHRS.h"
#include "SFDrive.h"

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
        const int cubeManipAngleNum = 1;
        const int cubeManipAngleLimitNum = 1;
        //Motor tuning constants
        double scale = 1;
        const double TICKS_PER_INCH = 217.3;
        double pConstantDrive = .1;
        double iConstantDrive = 0.001;
        double dConstantDrive = 0;
        const double MAX_ANGLE_TICKS = 1000;
        double pConstantAngle = .1;
        double iConstantAngle = 0.001;
        double dConstantAngle = 0;
        double setPointDrive = 0;
        double setPointAngle = MAX_ANGLE_TICKS;
        //Misc
        int checkTimeout = 0;
        int timeOut = 100;
        int packetsReceived = 0;
        //booleans for what position you start in
        bool left = false;
        bool right = false;
        bool center = false;

        std::string gameData;
        std::string SwitchColor = gameData.substr(1,1);

    private:
        //Initialize variables
        AHRS * ahrs; //= new AHRS (SPI::Port::kMXP);
        WPI_TalonSRX * _lMotorFront = new WPI_TalonSRX (lMotorFrontNum);
        WPI_TalonSRX * _lMotorBack = new WPI_TalonSRX (lMotorBackNum);
        WPI_TalonSRX * _rMotorFront = new WPI_TalonSRX (rMotorFrontNum);
        WPI_TalonSRX * _rMotorBack = new WPI_TalonSRX (rMotorBackNum);
        Spark * _lCubeIntake = new Spark (lCubeIntakeNum);
        Spark * _rCubeIntake = new Spark (rCubeIntakeNum);
        WPI_TalonSRX * _cubeManipAngle = new WPI_TalonSRX (cubeManipAngleNum);

        Counter * cubeAngleLimit = new Counter (new DigitalInput (cubeManipAngleLimitNum));
        SFDrive *myRobot = new SFDrive (_lMotorFront, _rMotorFront, ahrs);
        Joystick *stick = new Joystick (joystickNum);

        void RobotInit ()
        {
            //Set back motors to follower mode
            _rMotorBack->Set(ctre::phoenix::motorcontrol::ControlMode::Follower, rMotorFrontNum);
            _lMotorBack->Set(ctre::phoenix::motorcontrol::ControlMode::Follower, lMotorFrontNum);

            //used for inverting motors
            _rMotorFront->SetSensorPhase (true);
            _rMotorBack->SetSensorPhase (true);
            _lMotorFront->SetSensorPhase (true);
            _lMotorBack->SetSensorPhase (true);
            _cubeManipAngle->SetSensorPhase (false);

            //used to config the motor controllers for QuadEncoders(type of encoder)
            ctre::phoenix::motorcontrol::FeedbackDevice qE = QuadEncoder;
            _lMotorFront->ConfigSelectedFeedbackSensor (qE, 0, checkTimeout);
            _lMotorBack->ConfigSelectedFeedbackSensor (qE, 0, checkTimeout);
            _rMotorFront->ConfigSelectedFeedbackSensor (qE, 0, checkTimeout);
            _rMotorBack->ConfigSelectedFeedbackSensor (qE, 0, checkTimeout);
            _cubeManipAngle->ConfigSelectedFeedbackSensor (qE, 0, checkTimeout);

            _rMotorFront->SelectProfileSlot (0, 0);
            _rMotorBack->SelectProfileSlot (0, 0);
            _lMotorFront->SelectProfileSlot (0, 0);
            _lMotorBack->SelectProfileSlot (0, 0);
            _cubeManipAngle->SelectProfileSlot (0, 0);

            cubeAngleLimit->Reset ();
        }

        void TeleopInit ()
        {
            DriverStation::ReportError("TeleopInit Started");
            //Set encoder positions to 0
            _rMotorFront->GetSensorCollection ().SetQuadraturePosition (0, checkTimeout);
            _rMotorBack->GetSensorCollection ().SetQuadraturePosition (0, checkTimeout);
            _lMotorFront->GetSensorCollection ().SetQuadraturePosition (0, checkTimeout);
            _lMotorBack->GetSensorCollection ().SetQuadraturePosition (0, checkTimeout);
            myRobot->ArcadeDrive (0.0, 0.0);
            DriverStation::ReportError("TeleopInit Completed");
        }

        void TeleopPeriodic ()
        {
            myRobot->ArcadeDrive (scale * stick->GetRawAxis (1), -(stick->GetRawAxis (4) > 0 ? 1 : -1) * stick->GetRawAxis (4) * stick->GetRawAxis (4));

            if(stick->GetRawAxis(3) > TRIGGER_DEADZONE)//right trigger
            {
            	this->_lCubeIntake->Set(stick->GetRawAxis(3));
            	this->_rCubeIntake->Set(stick->GetRawAxis(3));
            }
            else if(stick->GetRawAxis(2) > TRIGGER_DEADZONE)//left trigger
            {
            	this->_lCubeIntake->Set(-stick->GetRawAxis(2));
            	this->_rCubeIntake->Set(-stick->GetRawAxis(2));
            }
            else
            {
             	this->_lCubeIntake->Set(0);
                this->_rCubeIntake->Set(0);
            }
            if(stick->GetRawButton(1)) // a button
            {
            	this->_cubeManipAngle->Set(1);
            }
            else if(stick->GetRawButton(4)) // y button
            {
            	this->_cubeManipAngle->Set(-1);

            }
            else{
            	this->_cubeManipAngle->Set(0);
            }
        }

        void AutonomousInit ()
        {
            DriverStation::ReportError("AutonInit Started");
            ConfigPIDS ();
            DriverStation::ReportError("AutonInit Completed");
        }

        void AutonomousPeriodic ()
        {

        }

        void TestInit ()
        {
            DriverStation::ReportError("TestInit Started");
            ConfigPIDS ();
            //Put PID values in ShuffleBoard
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
            DriverStation::ReportError("TestInit Completed");
        }

        void TestPeriodic ()
        {
            if (packetsReceived % 100 == 0) //Update PID and setpoint values from shuffleboard
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
            }

            packetsReceived++;
            if (packetsReceived == 200)
                myRobot->PIDDrive (setPointDrive, setPointDrive);
            if (setPointAngle < MAX_ANGLE_TICKS && setPointAngle > 0)
                _cubeManipAngle->Set (ctre::phoenix::motorcontrol::ControlMode::Position, setPointAngle);

        }

        void ConfigPIDS ()
        {
            DriverStation::ReportError("PID Config Started");
            _rMotorFront->GetSensorCollection ().SetQuadraturePosition (0, checkTimeout);
            _rMotorBack->GetSensorCollection ().SetQuadraturePosition (0, checkTimeout);
            _lMotorFront->GetSensorCollection ().SetQuadraturePosition (0, checkTimeout);
            _lMotorBack->GetSensorCollection ().SetQuadraturePosition (0, checkTimeout);

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

            //Raise angle motor until limit is triggered, and then set position to what it is when maxed
             /*while (!cubeAngleLimit->Get ())
             {
                 _cubeManipAngle->Set (0.1);
             }
             cubeAngleLimit->Reset ();
             _cubeManipAngle->GetSensorCollection ().SetQuadraturePosition (MAX_ANGLE_TICKS, checkTimeout);
            */
            /*
             * NOTE - RIGHT NOW THIS IS SUPPOSED TO HAPPEN SUPER SLOWLY
             * THIS IS BECAUSE, AFTER THE LIMIT SWITCH IS INSTALLED, IT MAY NOT BE SET UP CORRECTLY
             * SOMEONE SHOULD WATCH THE MANIPULATOR AND MAKE SURE THE SWITCH IS ACTUALLY TRIGGERED
             * IF THE MANIPULATOR GOES PAST ITS PATH OF TRAVEL, E-STOP THE ROBOT
             * AT COMPETITION, THIS SHOULD BE SET TO FULL SPEED
             */
            setPointAngle = MAX_ANGLE_TICKS;
            setPointDrive = 0;
            DriverStation::ReportError("PID Config Completed");
        }
        void move(double inches){
        	double NumOfTicks = inches * TICKS_PER_INCH;
        	_lMotorFront->WPI_TalonSRX::Set(ctre::phoenix::motorcontrol::ControlMode::MotionProfile, NumOfTicks);
        	_rMotorFront->WPI_TalonSRX::Set(ctre::phoenix::motorcontrol::ControlMode::MotionProfile, NumOfTicks);
        	_lMotorBack->WPI_TalonSRX::Set(ctre::phoenix::motorcontrol::ControlMode::Follower, rMotorFrontNum);
        	_rMotorBack->WPI_TalonSRX::Set(ctre::phoenix::motorcontrol::ControlMode::Follower, lMotorFrontNum);
        }
    	void nothing(){
    		move(0.0);
        }
    	void Easy(){
    		if(left){
    			move(140);
    		}
    		if(right){
    			move(140(;))
    		}
    		if(center){
    			if(SwitchColor == "r"){
    				move(48.5);
    				myRobot->PIDTurn(90);
    				move(71.23);
    				myRobot->PIDTurn(270);
    				move(73);

    		}
    	}
};

START_ROBOT_CLASS(Robot)
