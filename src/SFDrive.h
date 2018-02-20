#include <ctre/Phoenix.h>
#include "ahrs.h"

#ifndef SRC_SFDRIVE_H_
#define SRC_SFDRIVE_H_

class SFDrive
{
private: //MEMBER VARIABLES
	WPI_TalonSRX * m_leftMotor;
	WPI_TalonSRX * m_rightMotor;
	AHRS * m_ahrs;
	double m_deadband = 0.08;
	double m_lastPIDTime = 0;
	int m_PIDStepTime = 10 ^ 8;
	double m_PIDStepSize = 500;

public:
	SFDrive(WPI_TalonSRX * lMotor, WPI_TalonSRX * rMotor, AHRS * ahrs = NULL);
	void ArcadeDrive(double xSpeed, double zRotation);
	void PIDDrive(double _rMotorSet, double _lMotorSet = _rMotorSet);
	void PIDTurn(double degreesClockwise);
};

#endif
