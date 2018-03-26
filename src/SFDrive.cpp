#include <SFDrive.h>
#include <ctre/Phoenix.h>
#include <Timer.h>
#include <AnalogGyro.h>

using namespace frc;

SFDrive::SFDrive(WPI_TalonSRX * lMotor, WPI_TalonSRX * rMotor, AnalogGyro * _gyro) :
                m_leftMotor(lMotor), m_rightMotor(rMotor), gyro(_gyro){ }

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

void GyroTurn(double degrees)
{

}
