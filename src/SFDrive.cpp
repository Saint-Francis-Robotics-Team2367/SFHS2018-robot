#include <SFDrive.h>
#include <ctre/Phoenix.h>

SFDrive::SFDrive(WPI_TalonSRX * lMotor, WPI_TalonSRX * rMotor) :
		m_leftMotor(lMotor), m_rightMotor(rMotor) {

}

void SFDrive::ArcadeDrive(double xSpeed, double zRotation) {
	double leftMotorOutput;
	double rightMotorOutput;

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

	m_leftMotor->Set(leftMotorOutput);
	m_rightMotor->Set(-rightMotorOutput);

}
