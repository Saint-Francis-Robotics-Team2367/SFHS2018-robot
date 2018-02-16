#include <ctre/Phoenix.h>
#include "ahrs.h"

#ifndef SRC_SFDRIVE_H_
#define SRC_SFDRIVE_H_

class SFDrive
{
private: //MEMBER VARIABLES
	WPI_TalonSRX * m_leftMotorFront;
	WPI_TalonSRX * m_leftMotorBack;
	WPI_TalonSRX * m_rightMotorFront;
        WPI_TalonSRX * m_rightMotorBack;
	AHRS * m_ahrs;
	double m_deadband = 0.08;

public:
	SFDrive(WPI_TalonSRX * lMotorFront, WPI_TalonSRX * lMotorBack, WPI_TalonSRX * rMotorFront, WPI_TalonSRX * rMotorBack, AHRS * ahrs = NULL);
	void ArcadeDrive(double xSpeed, double zRotation);
	void PIDDrive(double _rMotorSet, double _lMotorSet);

};

#endif
