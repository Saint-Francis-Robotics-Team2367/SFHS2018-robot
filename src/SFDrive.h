#include <ctre/Phoenix.h>

#ifndef SRC_SFDRIVE_H_
#define SRC_SFDRIVE_H_

class SFDrive
{
private: //MEMBER VARIABLES
	WPI_TalonSRX * m_leftMotor;
	WPI_TalonSRX * m_rightMotor;

public:
	SFDrive(WPI_TalonSRX * lMotor, WPI_TalonSRX * rMotor);
	void ArcadeDrive(double xSpeed, double zRotation);
};

#endif
