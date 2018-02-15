/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

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

class Robot : public frc::IterativeRobot
{
    public:
        const int joystickNum = 0;
        const int rMotorNum = 4;
        const int lMotorNum = 1;
        double scale = 1;
        const double TICKS_PER_INCH = 325.95;
        double pConstant = .1;
        double iConstant = 0.001;
        double dConstant = 0;
        int setPoint = 0;
        int checkTimeout = 0;
        int timeOut = 100;
        int packetsReceived = 0;
    private:
        AHRS * ahrs = new AHRS (SerialPort::kMXP);

        WPI_TalonSRX * _rMotor = new WPI_TalonSRX (rMotorNum);
        WPI_TalonSRX * _lMotor = new WPI_TalonSRX (lMotorNum);

        DifferentialDrive *myRobot = new DifferentialDrive (*_lMotor, *_rMotor);
        Joystick *stick = new Joystick (joystickNum);

        void RobotInit ()
        {
            //used to config the motor controllers for QuadEncoders(type of encoder)
            ctre::phoenix::motorcontrol::FeedbackDevice qE = QuadEncoder;
            _lMotor->ConfigSelectedFeedbackSensor (qE, 0, checkTimeout);
            _rMotor->ConfigSelectedFeedbackSensor (qE, 0, checkTimeout);

            //used for inverting motors
            _rMotor->SetSensorPhase (true);
            _lMotor->SetSensorPhase (true);

            _rMotor->SelectProfileSlot(0,0);
            _lMotor->SelectProfileSlot(0,0);
        }

        void TeleopInit ()
        {
            _rMotor->GetSensorCollection().SetQuadraturePosition(0, checkTimeout);
            _lMotor->GetSensorCollection().SetQuadraturePosition(0, checkTimeout);
            myRobot->ArcadeDrive (0.0, 0.0);
        }

        void TeleopPeriodic ()
        {
            myRobot->ArcadeDrive (scale * stick->GetRawAxis (1), (stick->GetRawAxis (4) > 0 ? 1 : -1) * stick->GetRawAxis (4) * stick->GetRawAxis (4));
        }

        void AutonomousInit ()
        {
            _rMotor->GetSensorCollection().SetQuadraturePosition(0, checkTimeout);
            _lMotor->GetSensorCollection().SetQuadraturePosition(0, checkTimeout);
        }

        void AutonomousPeriodic ()
        {

        }

        void TestInit ()
        {
            SmartDashboard::PutNumber ("P", pConstant);
            SmartDashboard::PutNumber ("I", iConstant);
            SmartDashboard::PutNumber ("D", dConstant);
            SmartDashboard::PutNumber ("Setpoint", setPoint);
            SmartDashboard::PutNumber("Current Position - Right", 0);
            SmartDashboard::PutNumber("Current Position - Left", 0);

            _rMotor->GetSensorCollection().SetQuadraturePosition(0, checkTimeout);
            _lMotor->GetSensorCollection().SetQuadraturePosition(0, checkTimeout);
        }

        void TestPeriodic ()
        {
            packetsReceived++;
            if (packetsReceived % 100 == 0) //Update PID and setpoint values from shuffleboard
            {
                pConstant = SmartDashboard::GetNumber ("P", pConstant);
                iConstant = SmartDashboard::GetNumber ("I", pConstant);
                dConstant = SmartDashboard::GetNumber ("D", pConstant);
                setPoint = SmartDashboard::GetNumber ("Setpoint", setPoint);

            }

            _lMotor->Config_kP (0, pConstant, checkTimeout);
            _lMotor->Config_kI (0, iConstant, checkTimeout);
            _lMotor->Config_kD (0, dConstant, checkTimeout);

            _rMotor->Config_kP (0, pConstant, checkTimeout);
            _rMotor->Config_kI (0, iConstant, checkTimeout);
            _rMotor->Config_kD (0, dConstant, checkTimeout);

            _rMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, setPoint);
            _lMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, setPoint);
        }
};

START_ROBOT_CLASS(Robot)
