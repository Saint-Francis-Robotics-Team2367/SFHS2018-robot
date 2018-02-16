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

class Robot : public frc::IterativeRobot
{
    public:
        const int joystickNum = 0;
        const int rMotorFrontNum = 5;
        const int rMotorBackNum = 4;
        const int lMotorFrontNum = 3;
        const int lMotorBackNum = 2;
        double scale = 1;
        const double TICKS_PER_INCH = 325.95;
        double pConstant = .1;
        double iConstant = 0.001;
        double dConstant = 0;
        int checkTimeout = 0;
        int timeOut = 100;
        int packetsReceived = 0;
    private:
        AHRS * ahrs;
        WPI_TalonSRX * _lMotorFront = new WPI_TalonSRX (lMotorFrontNum);
        WPI_TalonSRX * _lMotorBack = new WPI_TalonSRX (lMotorBackNum);
        WPI_TalonSRX * _rMotorFront = new WPI_TalonSRX (rMotorFrontNum);
        WPI_TalonSRX * _rMotorBack = new WPI_TalonSRX (rMotorBackNum);

        SFDrive *myRobot = new SFDrive (_lMotorFront, _lMotorBack, _rMotorFront, _rMotorBack, ahrs);
        Joystick *stick = new Joystick (joystickNum);

        void RobotInit ()
        {
            //used to config the motor controllers for QuadEncoders(type of encoder)
            ctre::phoenix::motorcontrol::FeedbackDevice qE = QuadEncoder;
            _lMotorFront->ConfigSelectedFeedbackSensor (qE, 0, checkTimeout);
            _lMotorBack->ConfigSelectedFeedbackSensor (qE, 0, checkTimeout);
            _rMotorFront->ConfigSelectedFeedbackSensor (qE, 0, checkTimeout);
            _rMotorBack->ConfigSelectedFeedbackSensor (qE, 0, checkTimeout);

            //used for inverting motors
            _rMotorFront->SetSensorPhase(true);
            _rMotorBack->SetSensorPhase(true);
            _lMotorFront->SetSensorPhase(true);
            _lMotorBack->SetSensorPhase(true);

            _rMotorFront->SelectProfileSlot (0, 0);
            _rMotorBack->SelectProfileSlot (0, 0);
            _lMotorFront->SelectProfileSlot (0, 0);
            _lMotorBack->SelectProfileSlot (0, 0);
        }

        void TeleopInit ()
        {
            _rMotorFront->GetSensorCollection ().SetQuadraturePosition (0, checkTimeout);
            _rMotorBack->GetSensorCollection ().SetQuadraturePosition (0, checkTimeout);
            _lMotorFront->GetSensorCollection ().SetQuadraturePosition (0, checkTimeout);
            _lMotorBack->GetSensorCollection ().SetQuadraturePosition (0, checkTimeout);
            myRobot->ArcadeDrive (0.0, 0.0);
        }

        void TeleopPeriodic ()
        {
            myRobot->ArcadeDrive (scale * stick->GetRawAxis (1), -(stick->GetRawAxis (4) > 0 ? 1 : -1) * stick->GetRawAxis (4) * stick->GetRawAxis (4));
        }

        void AutonomousInit ()
        {
            _rMotorFront->GetSensorCollection ().SetQuadraturePosition (0, checkTimeout);
            _rMotorBack->GetSensorCollection ().SetQuadraturePosition (0, checkTimeout);
            _lMotorFront->GetSensorCollection ().SetQuadraturePosition (0, checkTimeout);
            _lMotorBack->GetSensorCollection ().SetQuadraturePosition (0, checkTimeout);
        }

        void AutonomousPeriodic ()
        {

        }

        void TestInit ()
        {
            SmartDashboard::PutNumber ("P", pConstant);
            SmartDashboard::PutNumber ("I", iConstant);
            SmartDashboard::PutNumber ("D", dConstant);
            SmartDashboard::PutNumber ("Setpoint", 0);
            SmartDashboard::PutNumber ("Current Position - Right", 0);
            SmartDashboard::PutNumber ("Current Position - Left", 0);

            _rMotorFront->GetSensorCollection ().SetQuadraturePosition (0, checkTimeout);
            _lMotorFront->GetSensorCollection ().SetQuadraturePosition (0, checkTimeout);
        }

        void TestPeriodic ()
        {
            if (packetsReceived % 100 == 0) //Update PID and setpoint values from shuffleboard
            {
                pConstant = SmartDashboard::GetNumber ("P", pConstant);
                iConstant = SmartDashboard::GetNumber ("I", pConstant);
                dConstant = SmartDashboard::GetNumber ("D", pConstant);
                SmartDashboard::PutNumber ("Current Position - Right", _rMotorFront->GetSensorCollection ().GetQuadraturePosition ());
                SmartDashboard::PutNumber ("Current Position - Left", _lMotorFront->GetSensorCollection ().GetQuadraturePosition ());
                _lMotorFront->Config_kP (0, pConstant, checkTimeout);
                _lMotorFront->Config_kI (0, iConstant, checkTimeout);
                _lMotorFront->Config_kD (0, dConstant, checkTimeout);
                _lMotorBack->Config_kP (0, pConstant, checkTimeout);
                _lMotorBack->Config_kI (0, iConstant, checkTimeout);
                _lMotorBack->Config_kD (0, dConstant, checkTimeout);
                _rMotorFront->Config_kP (0, pConstant, checkTimeout);
                _rMotorFront->Config_kI (0, iConstant, checkTimeout);
                _rMotorFront->Config_kD (0, dConstant, checkTimeout);
                _rMotorBack->Config_kP (0, pConstant, checkTimeout);
                _rMotorBack->Config_kI (0, iConstant, checkTimeout);
                _rMotorBack->Config_kD (0, dConstant, checkTimeout);
            }
            packetsReceived++;
            myRobot->PIDDrive (SmartDashboard::GetNumber ("Setpoint", 0), SmartDashboard::GetNumber ("Setpoint", 0));
        }
};

START_ROBOT_CLASS(Robot)
