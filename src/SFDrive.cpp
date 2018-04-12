#include <SFDrive.h>
#include <ctre/Phoenix.h>
#include <Timer.h>
#include <cmath>

using namespace frc;

SFDrive::SFDrive(WPI_TalonSRX * lMotor, WPI_TalonSRX * rMotor) :
                m_leftMotor(lMotor), m_rightMotor(rMotor) { }

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

bool SFDrive::PIDDrive(float inches, float maxVel, float timeout, bool ZeroVelocityAtEnd)
{

	int setPoint = 0;
	double  startTime ,currStepTime, lastStepTime, deltaTime;

	//convert from inches to encoder ticks
	float endPoint = abs(inches) / m_wheelCircumference * m_ticksPerRev;
	int maxVelDelta = maxVel/m_wheelCircumference * m_ticksPerRev;

	//zero encoder
	disableP();
	m_leftMotor->SetSelectedSensorPosition(0,0,m_canTimeout);
	m_rightMotor->SetSelectedSensorPosition(0,0,m_canTimeout);
	m_leftMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, 0);
	m_rightMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position,0);
	enableP();

	if(ZeroVelocityAtEnd)
	{
		startTime = lastStepTime = Timer().GetFPGATimestamp();
		while(setPoint != endPoint && startTime + timeout > lastStepTime)
		{
			//handle timing
			currStepTime = Timer().GetFPGATimestamp();
			deltaTime = currStepTime - lastStepTime;
			lastStepTime = currStepTime;


			if(endPoint - setPoint < (m_currVelocity *m_currVelocity) /(2*m_maxAccl)) //check if you decelerated at the current speed
			{																		  //would you hit 0 before you reached the end point
				m_currVelocity -= m_maxAccl * deltaTime;
				if(m_currVelocity < m_minDecelVel) m_currVelocity = m_minDecelVel;
			}
		else																			//otherwise accelerate
			{
				m_currVelocity += m_maxAccl * deltaTime;
				if(m_currVelocity > maxVelDelta) m_currVelocity = maxVelDelta;
			}

			setPoint += m_currVelocity * deltaTime;
			if(setPoint > endPoint) setPoint = endPoint;

			m_leftMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, std::copysign(setPoint, inches));
			m_rightMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position,std::copysign(setPoint, inches) * -1);
		}
	}
	else
	{
		startTime = lastStepTime = Timer().GetFPGATimestamp();
		while(setPoint != endPoint && startTime + timeout > lastStepTime)
		{
			//handle timing
			currStepTime = Timer().GetFPGATimestamp();
			deltaTime = currStepTime - lastStepTime;
			lastStepTime = currStepTime;


			m_currVelocity += m_maxAccl * deltaTime;
			if(m_currVelocity > maxVelDelta) m_currVelocity = maxVelDelta;
			setPoint += m_currVelocity * deltaTime;
			if(setPoint > endPoint) setPoint = endPoint;

			m_leftMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, std::copysign(setPoint, inches));
			m_rightMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position,std::copysign(setPoint, inches) * -1);
		}
	}

	if(lastStepTime > startTime + timeout) //simple error check, did we finish the motion before we ran out of time
	{
		return false;
	}
	else
	{
		return true;
	}
}

bool  SFDrive::PIDTurn(float degreesClockwise, float radius, float maxVel, float timeout, bool ZeroVelocityAtEnd)
{

	int setPoint = 0, endPoint, innerChordLen;
	float endAngle = abs(degreesClockwise);
	int maxVelDelta = maxVel/m_wheelCircumference * m_ticksPerRev;
	double  startTime ,currStepTime, lastStepTime, deltaTime;

	//zero encoder
	disableP();
	m_leftMotor->SetSelectedSensorPosition(0,0,m_canTimeout);
	m_rightMotor->SetSelectedSensorPosition(0,0,m_canTimeout);
	m_leftMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, 0);
	m_rightMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position,0);
	enableP();


	if(radius < m_wheelTrack/2) radius = m_wheelTrack/2.0f;
	endPoint = (((endAngle / 360.0f) * (radius+ m_wheelTrack/2.0f * 2* M_PI)) /m_wheelCircumference) * m_ticksPerRev;
	innerChordLen = (((endAngle / 360.0f) * (radius - m_wheelTrack/2.0f * 2* M_PI)) /m_wheelCircumference) * m_ticksPerRev;

	startTime = lastStepTime = Timer().GetFPGATimestamp();

	if(ZeroVelocityAtEnd)
	{
		startTime = lastStepTime = Timer().GetFPGATimestamp();
		while(setPoint != endPoint && startTime + timeout > lastStepTime)
		{
			//handle timing
			currStepTime = Timer().GetFPGATimestamp();
			deltaTime = currStepTime - lastStepTime;
			lastStepTime = currStepTime;


			if(endPoint - setPoint < (m_currVelocity *m_currVelocity) /(2*m_maxAccl)) //check if you decelerated at the current speed
			{																		  //would you hit 0 before you reached the end point
				m_currVelocity -= m_maxAccl * deltaTime;
				if(m_currVelocity < m_minDecelVel) m_currVelocity = m_minDecelVel;
			}
		else																			//otherwise accelerate
			{
				m_currVelocity += m_maxAccl * deltaTime;
				if(m_currVelocity > maxVelDelta) m_currVelocity = maxVelDelta;
			}

			setPoint += m_currVelocity * deltaTime;
			if(setPoint > endPoint) setPoint = endPoint;
			int innerSet = ((float) setPoint/ (float)endPoint) * innerChordLen;

			if(degreesClockwise > 0)
			{
				m_leftMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, setPoint);
				m_rightMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position,innerSet *-1);
			}
			else
			{
				m_leftMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, innerSet);
				m_rightMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position,setPoint * -1);
			}
		}
	}
	else
	{
		while(setPoint < endPoint && startTime + timeout > lastStepTime)
		{
			//handle timing
			currStepTime = Timer().GetFPGATimestamp();
			deltaTime = currStepTime - lastStepTime;
			lastStepTime = currStepTime;

			//move
			m_currVelocity += m_maxAccl * deltaTime;
			if(m_currVelocity > maxVelDelta) m_currVelocity = maxVelDelta;
			setPoint += m_currVelocity * deltaTime;
			if(setPoint > endPoint) setPoint = endPoint;
			int innerSet = ((float) setPoint/ (float)endPoint) * innerChordLen;

			if(degreesClockwise > 0)
			{
				m_leftMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, setPoint);
				m_rightMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position,innerSet *-1);
			}
			else
			{
				m_leftMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, innerSet);
				m_rightMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position,setPoint * -1);
			}
		}
	}

	if(lastStepTime > startTime + timeout) //simple error check, did we finish the motion before we ran out of time
	{
		return false;
	}
	else
	{
		return true;
	}
}

bool SFDrive::PIDShoot(float moveInches, float shootStartDist, float shootTime, float maxVel, float timeout)
{
	int setPoint = 0;
	double  startTime ,currStepTime, lastStepTime, deltaTime, shootStartTime = 0;

	//convert from inches to encoder ticks
	float endPoint = abs(moveInches) / m_wheelCircumference * m_ticksPerRev;
	float shootPoint = abs(shootStartDist) / m_wheelCircumference * m_ticksPerRev;
	int maxVelDelta = maxVel/m_wheelCircumference * m_ticksPerRev;

	//zero encoder
	disableP();
	m_leftMotor->SetSelectedSensorPosition(0,0,m_canTimeout);
	m_rightMotor->SetSelectedSensorPosition(0,0,m_canTimeout);
	m_leftMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, 0);
	m_rightMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position,0);
	enableP();

	startTime = lastStepTime = Timer().GetFPGATimestamp();
	while(setPoint != endPoint && startTime + timeout > lastStepTime)
	{
		//handle timing
		currStepTime = Timer().GetFPGATimestamp();
		deltaTime = currStepTime - lastStepTime;
		lastStepTime = currStepTime;


		m_currVelocity += m_maxAccl * deltaTime;
		if(m_currVelocity > maxVelDelta) m_currVelocity = maxVelDelta;
		setPoint += m_currVelocity * deltaTime;
		if(setPoint > endPoint) setPoint = endPoint;

		if(setPoint >= shootPoint && shootStartTime == 0)
		{
			//shoot
			shootStartTime = Timer().GetFPGATimestamp();
		}
		if(Timer().GetFPGATimestamp() > shootStartTime + shootTime)
		{
			//stop Shot
		}

		m_leftMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, std::copysign(setPoint, moveInches));
		m_rightMotor->Set(ctre::phoenix::motorcontrol::ControlMode::Position,std::copysign(setPoint, moveInches) * -1);
	}

	//stop Shot here as well in case the timeout cuts off the loop.

	if(lastStepTime > startTime + timeout && lastStepTime > shootStartTime + shootTime) //simple error check, did we finish the motion before we ran out of time
	{
		return false;
	}
	else
	{
		return true;
	}
}
void SFDrive::initPID()
{
	m_leftMotor->Config_kP (0, m_P, m_canTimeout);
	m_leftMotor->Config_kI (0, m_I, m_canTimeout);
	m_leftMotor->Config_kD (0, m_D, m_canTimeout);

	m_rightMotor->Config_kP (0, m_P, m_canTimeout);
	m_rightMotor->Config_kI (0, m_I, m_canTimeout);
	m_rightMotor->Config_kD (0, m_D, m_canTimeout);
}

void SFDrive::enableP()
{
	m_leftMotor->Config_kP (0, m_P, m_canTimeout);
	m_rightMotor->Config_kP (0, m_P, m_canTimeout);
}

void SFDrive::disableP()
{

	m_leftMotor->Config_kP (0, 0, m_canTimeout);
	m_rightMotor->Config_kP (0, 0, m_canTimeout);
}
void SFDrive::setAccel(float accl)
{
	m_maxAccl = accl;
}
