#include <SFDrive.h>
#include <ctre/Phoenix.h>

SFDrive::SFDrive(WPI_TalonSRX * lMotorFront, WPI_TalonSRX * lMotorBack, WPI_TalonSRX * rMotorFront, WPI_TalonSRX * rMotorBack, AHRS * ahrs) :
		m_leftMotorFront(lMotorFront), m_leftMotorBack(lMotorBack), m_rightMotorFront(rMotorFront), m_rightMotorBack(rMotorBack), m_ahrs(ahrs) { }

void SFDrive::ArcadeDrive(double xSpeed, double zRotation) {
	double leftMotorOutput;
	double rightMotorOutput;

	if(abs(xSpeed) < m_deadband)
	    xSpeed = 0;
	if(abs(zRotation) < m_deadband)
	    zRotation = 0;

	double maxInput = std::copysign(std::max(std::abs(xSpeed), std::abs(zRotation)), xSpeed);

	if (xSpeed >= 0.0) {
		// First quadrant, else second quadrant
		if (zRotation >= 0.0) {
			leftMotorOutput = maxInput;
			rightMotorOutput = xSpeed - zRotation;
		} else {
			leftMotorOutput = xSpeed + zRotation;
			rightMotorOutput = maxInput;
		}
	} else {
		// Third quadrant, else fourth quadrant
		if (zRotation >= 0.0) {
			leftMotorOutput = xSpeed + zRotation;
			rightMotorOutput = maxInput;
		} else {
			leftMotorOutput = maxInput;
			rightMotorOutput = xSpeed - zRotation;
		}
	}

	m_leftMotorFront->Set(leftMotorOutput);
	m_leftMotorBack->Set(leftMotorOutput);
	m_rightMotorFront->Set(-rightMotorOutput);
	m_rightMotorBack->Set(-rightMotorOutput);

}

void SFDrive::PIDDrive(double _lMotorSet, double _rMotorSet)
{
    m_leftMotorFront->Set(ctre::phoenix::motorcontrol::ControlMode::Position, _lMotorSet);
    m_leftMotorBack->Set(ctre::phoenix::motorcontrol::ControlMode::Position, _lMotorSet);
    m_rightMotorFront->Set(ctre::phoenix::motorcontrol::ControlMode::Position, _rMotorSet);
    m_rightMotorBack->Set(ctre::phoenix::motorcontrol::ControlMode::Position, _rMotorSet);
}
