#include <SFDrive.h>
#include <ctre/Phoenix.h>
#include <Timer.h>

using namespace frc;

SFDrive::SFDrive(WPI_TalonSRX * lMotor, WPI_TalonSRX * rMotor, AHRS * ahrs) :
		m_leftMotor(lMotor), m_rightMotor(rMotor), m_ahrs(ahrs) { }

void SFDrive::ArcadeDrive(double xSpeed, double zRotation) {
	double leftMotorOutput;
	double rightMotorOutput;

	if(fabs(xSpeed) <= m_deadband)
	    xSpeed = 0;
	if(fabs(zRotation) <= m_deadband)
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
	m_leftMotor->Set(leftMotorOutput);
	m_rightMotor->Set(-rightMotorOutput);
}

void SFDrive::PIDDrive(double _lMotorSet, double _rMotorSet)
{
    m_leftMotor->WPI_TalonSRX::Set(ctre::phoenix::motorcontrol::ControlMode::MotionProfile, _lMotorSet);
    m_rightMotor->WPI_TalonSRX::Set(ctre::phoenix::motorcontrol::ControlMode::MotionProfile, _rMotorSet);
    /*
    double currentRightSetpoint = m_rightMotor->GetSensorCollection().GetQuadraturePosition();
    double currentLeftSetpoint =  m_leftMotor->GetSensorCollection().GetQuadraturePosition();

    if(Timer::GetFPGATimestamp() - m_lastPIDTime >= m_PIDStepTime && (currentRightSetpoint != _rMotorSet || currentLeftSetpoint != _lMotorSet))
    {
        m_leftMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, fabs(currentLeftSetpoint - _lMotorSet) <= m_PIDStepSize? _lMotorSet: currentLeftSetpoint - _lMotorSet > 0? m_PIDStepSize:-m_PIDStepSize);
        m_rightMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, fabs(currentRightSetpoint - _rMotorSet) <= m_PIDStepSize? _rMotorSet: currentRightSetpoint - _rMotorSet > 0? m_PIDStepSize:-m_PIDStepSize);
    }
    */
}
