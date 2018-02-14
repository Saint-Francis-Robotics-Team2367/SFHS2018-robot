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

class Robot: public frc::IterativeRobot {
public:
	const int joystickNum = 0;
	const int rMotorNum = 4;
	const int lMotorNum = 1;
	const double scale = 1;
private:
	WPI_TalonSRX * _rMotor = new WPI_TalonSRX(rMotorNum);
	WPI_TalonSRX * _lMotor = new WPI_TalonSRX(lMotorNum);

	DifferentialDrive *myRobot = new DifferentialDrive(*_lMotor,*_rMotor );
	Joystick *stick = new Joystick(joystickNum);

	Faults _faults_L;
	Faults _faults_R;

	void TeleopInit() {
		myRobot->ArcadeDrive(0.0, 0.0);
	}


	void TeleopPeriodic() {
		myRobot->ArcadeDrive(scale * stick->GetRawAxis(1), (stick->GetRawAxis(4) > 0? 1:-1) * stick->GetRawAxis(4) * stick->GetRawAxis(4));
	}

	void AutonomousPeriodic() {
		//rMotor->Set(ControlMode::Current, 1);
		//lMotor->Set(ControlMode::Current, 1);
	}
};

START_ROBOT_CLASS(Robot)
