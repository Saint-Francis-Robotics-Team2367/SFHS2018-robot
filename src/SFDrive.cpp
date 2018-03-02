#include <SFDrive.h>
#include <ctre/Phoenix.h>
#include <Timer.h>
#include "Robot.cpp"
#include <WPILib.h>

using namespace frc;

SFDrive::SFDrive(WPI_TalonSRX * lMotor, WPI_TalonSRX * rMotor, AHRS * ahrs) :
		m_leftMotor(lMotor), m_rightMotor(rMotor), m_ahrs(ahrs) { }

//-----------------------------------------------------used to convert------------------------------------------------------------------------
enum Constants {

	/** which Talon on CANBus*/
	kTalonID = 0,

	/* example Victor SPX follower */
	kVictorFollower = 0,

	/**
	 * How many sensor units per rotation.
	 * Using CTRE Magnetic Encoder.
	 * @link https://github.com/CrossTheRoadElec/Phoenix-Documentation#what-are-the-units-of-my-sensor
	 */
	kSensorUnitsPerRotation = 4096,

	/**
	 * Which PID slot to pull gains from.  Starting 2018, you can choose
	 * from 0,1,2 or 3.  Only the first two (0,1) are visible in web-based configuration.
	 */
	kSlotIdx = 0,

	/**
	 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops.
	 * For now we just want the primary one.
	 */
	kPIDLoopIdx = 0,
	/**
	 * set to zero to skip waiting for confirmation, set to nonzero to wait
	 * and report to DS if action fails.
	 */
	kTimeoutMs = 10,

	/**
	 * Base trajectory period to add to each individual
	 * trajectory point's unique duration.  This can be set
	 * to any value within [0,255]ms.
	 */
	kBaseTrajPeriodMs = 0,

	/**
	 * Motor deadband, set to 1%.
	 */
	kNeutralDeadbandPercent = 1,
};

//------------------------------------------------------------used to convert between units----------------------------------------------------
const int kMotionProfileSz =185;
const double kMotionProfile[][3] = {
{0,	0	,10},
{4.76190476190476E-05,	0.571428571	,10},
{0.000214285714285714,	1.428571429	,10},
{0.000547619047619048,	2.571428571	,10},
{0.0010952380952381,	4	,10},
{0.0019047619047619,	5.714285714	,10},
{0.00302380952380952,	7.714285714	,10},
{0.0045,	10	,10},
{0.00638095238095238,	12.57142857	,10},
{0.00871428571428571,	15.42857143	,10},
{0.011547619047619,	18.57142857	,10},
{0.0149285714285714,	22	,10},
{0.0189047619047619,	25.71428571	,10},
{0.0235238095238095,	29.71428571	,10},
{0.0288333333333333,	34	,10},
{0.0348809523809524,	38.57142857	,10},
{0.0417142857142857,	43.42857143	,10},
{0.0493809523809524,	48.57142857	,10},
{0.0579285714285714,	54	,10},
{0.0674047619047619,	59.71428571	,10},
{0.0778571428571429,	65.71428571	,10},
{0.0893095238095238,	71.71428571	,10},
{0.101761904761905,	77.71428571	,10},
{0.115214285714286,	83.71428571	,10},
{0.129666666666667,	89.71428571	,10},
{0.145119047619048,	95.71428571	,10},
{0.161571428571429,	101.7142857	,10},
{0.17902380952381,	107.7142857	,10},
{0.19747619047619,	113.7142857	,10},
{0.216928571428571,	119.7142857	,10},
{0.237380952380952,	125.7142857	,10},
{0.258833333333333,	131.7142857	,10},
{0.281285714285714,	137.7142857	,10},
{0.304738095238095,	143.7142857	,10},
{0.329190476190476,	149.7142857	,10},
{0.354642857142857,	155.7142857	,10},
{0.381095238095238,	161.7142857	,10},
{0.408547619047619,	167.7142857	,10},
{0.437,	173.7142857	,10},
{0.466452380952381,	179.7142857	,10},
{0.496904761904762,	185.7142857	,10},
{0.528309523809524,	191.1428571	,10},
{0.560595238095238,	196.2857143	,10},
{0.593714285714286,	201.1428571	,10},
{0.627619047619048,	205.7142857	,10},
{0.662261904761905,	210	,10},
{0.697595238095238,	214	,10},
{0.733571428571429,	217.7142857	,10},
{0.770142857142857,	221.1428571	,10},
{0.807261904761905,	224.2857143	,10},
{0.844880952380952,	227.1428571	,10},
{0.882952380952381,	229.7142857	,10},
{0.921428571428571,	232	,10},
{0.960261904761905,	234	,10},
{0.999404761904762,	235.7142857	,10},
{1.03880952380952,	237.1428571	,10},
{1.07842857142857,	238.2857143	,10},
{1.11821428571429,	239.1428571	,10},
{1.15811904761905,	239.7142857	,10},
{1.19809523809524,	240	,10},
{1.23809523809524,	240	,10},
{1.27809523809524,	240	,10},
{1.31809523809524,	240	,10},
{1.35809523809524,	240	,10},
{1.39809523809524,	240	,10},
{1.43809523809524,	240	,10},
{1.47809523809524,	240	,10},
{1.51809523809524,	240	,10},
{1.55809523809524,	240	,10},
{1.59809523809524,	240	,10},
{1.63809523809524,	240	,10},
{1.67809523809524,	240	,10},
{1.71809523809524,	240	,10},
{1.75809523809524,	240	,10},
{1.79809523809524,	240	,10},
{1.83809523809524,	240	,10},
{1.87809523809524,	240	,10},
{1.91809523809524,	240	,10},
{1.95809523809524,	240	,10},
{1.99809523809524,	240	,10},
{2.03809523809524,	240	,10},
{2.07809523809524,	240	,10},
{2.11809523809524,	240	,10},
{2.15809523809524,	240	,10},
{2.19809523809524,	240	,10},
{2.23809523809524,	240	,10},
{2.27809523809524,	240	,10},
{2.31809523809524,	240	,10},
{2.35809523809524,	240	,10},
{2.39809523809524,	240	,10},
{2.43809523809524,	240	,10},
{2.47809523809524,	240	,10},
{2.51809523809524,	240	,10},
{2.55809523809524,	240	,10},
{2.59809523809524,	240	,10},
{2.63809523809524,	240	,10},
{2.67809523809524,	240	,10},
{2.71809523809524,	240	,10},
{2.75809523809524,	240	,10},
{2.79809523809524,	240	,10},
{2.83809523809524,	240	,10},
{2.87809523809524,	240	,10},
{2.91809523809524,	240	,10},
{2.95809523809524,	240	,10},
{2.99809523809524,	240	,10},
{3.03809523809524,	240	,10},
{3.07809523809524,	240	,10},
{3.11809523809524,	240	,10},
{3.15809523809524,	240	,10},
{3.19809523809524,	240	,10},
{3.23809523809524,	240	,10},
{3.27809523809524,	240	,10},
{3.31809523809524,	240	,10},
{3.35809523809524,	240	,10},
{3.39809523809524,	240	,10},
{3.43809523809524,	240	,10},
{3.47809523809524,	240	,10},
{3.51809523809524,	240	,10},
{3.55809523809524,	240	,10},
{3.59809523809524,	240	,10},
{3.63809523809524,	240	,10},
{3.67809523809524,	240	,10},
{3.71809523809524,	240	,10},
{3.75809523809524,	240	,10},
{3.79809523809524,	240	,10},
{3.83809523809524,	240	,10},
{3.87804761904762,	239.4285714	,10},
{3.91788095238095,	238.5714286	,10},
{3.95754761904762,	237.4285714	,10},
{3.997,	236	,10},
{4.03619047619048,	234.2857143	,10},
{4.07507142857143,	232.2857143	,10},
{4.11359523809524,	230	,10},
{4.15171428571429,	227.4285714	,10},
{4.18938095238095,	224.5714286	,10},
{4.22654761904762,	221.4285714	,10},
{4.26316666666667,	218	,10},
{4.29919047619048,	214.2857143	,10},
{4.33457142857143,	210.2857143	,10},
{4.36926190476191,	206	,10},
{4.40321428571429,	201.4285714	,10},
{4.43638095238095,	196.5714286	,10},
{4.46871428571429,	191.4285714	,10},
{4.50016666666667,	186	,10},
{4.53069047619048,	180.2857143	,10},
{4.5602380952381,	174.2857143	,10},
{4.58878571428572,	168.2857143	,10},
{4.61633333333334,	162.2857143	,10},
{4.64288095238095,	156.2857143	,10},
{4.66842857142857,	150.2857143	,10},
{4.69297619047619,	144.2857143	,10},
{4.71652380952381,	138.2857143	,10},
{4.73907142857143,	132.2857143	,10},
{4.76061904761905,	126.2857143	,10},
{4.78116666666667,	120.2857143	,10},
{4.80071428571429,	114.2857143	,10},
{4.81926190476191,	108.2857143	,10},
{4.83680952380953,	102.2857143	,10},
{4.85335714285714,	96.28571429	,10},
{4.86890476190476,	90.28571429	,10},
{4.88345238095238,	84.28571429	,10},
{4.897,	78.28571429	,10},
{4.90954761904762,	72.28571429	,10},
{4.92109523809524,	66.28571429	,10},
{4.93164285714286,	60.28571429	,10},
{4.94119047619048,	54.28571429	,10},
{4.94978571428572,	48.85714286	,10},
{4.9575,	43.71428571	,10},
{4.96438095238095,	38.85714286	,10},
{4.97047619047619,	34.28571429	,10},
{4.97583333333333,	30	,10},
{4.9805,	26	,10},
{4.98452380952381,	22.28571429	,10},
{4.98795238095238,	18.85714286	,10},
{4.99083333333334,	15.71428571	,10},
{4.99321428571429,	12.85714286	,10},
{4.99514285714286,	10.28571429	,10},
{4.99666666666667,	8	,10},
{4.99783333333334,	6	,10},
{4.99869047619048,	4.285714286	,10},
{4.99928571428572,	2.857142857	,10},
{4.99966666666667,	1.714285714	,10},
{4.99988095238095,	0.857142857	,10},
{4.99997619047619,	0.285714286	,10},
{5,	0	,10}};


//-----------------------------------------------------------variables for motionProfile------------------------------------------------------
	MotionProfileStatus _status;
	double _pos = 0, _vel = 0, _heading = 0;
	int _state = 0;
	int _loopTimeout = -1;
	bool _bStart = false;
	static const int kMinPointsInTalon = 5;
	static const int kNumLoopsTimeout = 10;
	SetValueMotionProfile _setValue = SetValueMotionProfile::Disable;
	WPI_TalonSRX * _talon;

//------------------------------------------------------------------------ArcadeDrive----------------------------------------------------------
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
//-------------------------------------------------------------------------PIDDrive------------------------------------------------------------
void SFDrive::PIDDrive(double _ticks)
{

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
//---------------------------------------------------reset---------------------------------------------------------------------
void reset()
{
	_talon->ClearMotionProfileTrajectories();
	_setValue = SetValueMotionProfile::Disable;
	_state = 0;
	_loopTimeout = -1;
	_bStart = false;
}
//---------------------------------------------------startFilling---------------------------------------------------------------
void startFilling(const double profile[][3], int totalCnt) {
		/* create an empty point */
		TrajectoryPoint point;

		/* did we get an underrun condition since last time we checked ? */
		if (_status.hasUnderrun) {
			/* better log it so we know about it */
			std::cout << "UNDERRUN\n";
			/*
			 * clear the error. This is what seperates "has underrun" from
			 * "is underrun", because the former is cleared by the application.
			 * That way, we never miss logging it.
			 */
			_talon->ClearMotionProfileHasUnderrun(Constants::kTimeoutMs);
		}

		/*
		 * just in case we are interrupting another MP and there is still buffer
		 * points in memory, clear it.
		 */
		_talon->ClearMotionProfileTrajectories();

		/* set the base trajectory period to zero, use the individual trajectory period below */
		_talon->ConfigMotionProfileTrajectoryPeriod(Constants::kBaseTrajPeriodMs, Constants::kTimeoutMs);

		/* This is fast since it's just into our TOP buffer */
		for (int i = 0; i < totalCnt; ++i) {
			double positionRot = profile[i][0];
			double velocityRPM = profile[i][1];

			/* for each point, fill our structure and pass it to API */
			point.position = positionRot * Constants::kSensorUnitsPerRotation; //Convert Revolutions to Units
			point.velocity = velocityRPM * Constants::kSensorUnitsPerRotation / 600.0; //Convert RPM to Units/100ms
			point.headingDeg = 0; /* future feature - not used in this example*/
			//point.ctre::phoenix::motion::MotionProfileStatus::profileSlotSelect0 = 0; /* which set of gains would you like to use [0,3]? */
			point.ctre::phoenix::motion::MotionProfileStatus::profileSlotSelect = 0;
			//point.profileSlotSelect1 = 0; /* future feature  - not used in this example - cascaded PID [0,1], leave zero */
			point.ctre::phoenix::motion::TrajectoryPoint::timeDur = GetTrajectoryDuration((int) profile[i][2]);
			point.ctre::phoenix::motion::
			point.zeroPos = false;
			if (i == 0)
				point.zeroPos = true; /* set this to true on the first point */

			point.isLastPoint = false;
			if ((i + 1) == totalCnt)
				point.isLastPoint = true; /* set this to true on the last point  */

			_talon->PushMotionProfileTrajectory(point);
		}
	}
//------------------------------------------------------------------control---------------------------------------------------------------------------
void control() {
		/*
		 * track time, this is rudimentary but that's okay, we just want to make
		 * sure things never get stuck.
		 */
		if (_loopTimeout < 0) {
			/* do nothing, timeout is disabled */
		} else {
			/* our timeout is nonzero */
			if (_loopTimeout == 0) {
				/*
				 * something is wrong. Talon is not present, unplugged, breaker
				 * tripped
				 */
				Instrumentation::OnNoProgress();
			} else {
				--_loopTimeout;
			}
		}

		/* first check if we are in MP mode */
		if (_talon->GetControlMode() != ControlMode::MotionProfile) {
			/*
			 * we are not in MP mode. We are probably driving the robot around
			 * using gamepads or some other mode.
			 */
			_state = 0;
			_loopTimeout = -1;
		} else {
			/*
			 * we are in MP control mode. That means: starting Mps, checking Mp
			 * progress, and possibly interrupting MPs if thats what you want to
			 * do.
			 */
			switch (_state) {
				case 0: /* wait for application to tell us to start an MP */
					if (_bStart) {
						_bStart = false;

						_setValue = SetValueMotionProfile::Disable;
						startFilling(kMotionProfile, kMotionProfileSz);

						/*
						 * MP is being sent to CAN bus, wait a small amount of time
						 */
						_state = 1;
						_loopTimeout = kNumLoopsTimeout;
					}
					break;
				case 1:
					/*
					 * wait for MP to stream to Talon, really just the first few
					 * points
					 */
					/* do we have a minimum numberof points in Talon */
					if (_status.btmBufferCnt > kMinPointsInTalon) {
						/* start (once) the motion profile */
						_setValue = SetValueMotionProfile::Enable;
						/* MP will start once the control frame gets scheduled */
						_state = 2;
						_loopTimeout = kNumLoopsTimeout;
					}
					break;
				case 2: /* check the status of the MP */
					/*
					 * if talon is reporting things are good, keep adding to our
					 * timeout. Really this is so that you can unplug your talon in
					 * the middle of an MP and react to it.
					 */
					if (_status.isUnderrun == false) {
						_loopTimeout = kNumLoopsTimeout;
					}
					/*
					 * If we are executing an MP and the MP finished, start loading
					 * another. We will go into hold state so robot servo's
					 * position.
					 */
					if (_status.activePointValid && _status.isLast) {
						/*
						 * because we set the last point's isLast to true, we will
						 * get here when the MP is done
						 */
						_setValue = SetValueMotionProfile::Hold;
						_state = 0;
						_loopTimeout = -1;
					}
					break;
			}

			/* Get the motion profile status every loop */
			_talon->GetMotionProfileStatus(_status);
			_heading = _talon->GetActiveTrajectoryHeading();
			_pos = _talon->GetActiveTrajectoryPosition();
			_vel = _talon->GetActiveTrajectoryVelocity();

			/* printfs and/or logging */
			Instrumentation::Process(_status, _pos, _vel, _heading);
		}
	}
