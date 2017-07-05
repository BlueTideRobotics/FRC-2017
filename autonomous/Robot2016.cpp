#include "WPILib.h"
#include "PID.h"
#include "SageFunctions.h"

/*
 * NAME IDEAS
 *
 * Thor
 * Goldfinger
 * White Knight
 * The GOP
 * Midas
 * Meme Machine
 */

class Robot: public IterativeRobot
{
private:
	Joystick stick;

	CANTalon frontLeft, frontRight, rearLeft, rearRight;
	RobotDrive myRobot;

	CANTalon sageArm;
	float sageArmSetVal;

	CANTalon winch1, winch2;
	float winchSetVal;

	CANTalon whip;
	float whipSetVal;

	CANJaguar intake, shooter; // CHANGE TO CANTalon at competition
	float intakeSetVal, shooterSetVal;

	Timer shooterTimer;
	float topPower, bottomPower;
	bool longShootingNow, shortShootingNow;
	Servo shootServo;

	Timer autonTimer;

	AnalogGyro gyro;
	PID gyroPID;
	bool gyroPIDrunning;
	double gyroPIDoutput;

	double centerX, centerY;
	HSLImage *targetImage;
	AxisCamera *cam;
public:
	Robot():
		stick(0),

		frontLeft(17),
		frontRight(16),
		rearLeft(13),
		rearRight(14),

		myRobot(frontLeft, rearLeft, frontRight, rearRight),

		sageArm(20),
		sageArmSetVal(0.0),

		winch1(18),
		winch2(19),
		winchSetVal(0.0),

		whip(15),
		whipSetVal(0.0),

		intake(11),
		shooter(10),
		intakeSetVal(0.0),
		shooterSetVal(0.0),

		shooterTimer(),
		topPower(0.0),
		bottomPower(0.0),
		longShootingNow(false),
		shortShootingNow(false),
		shootServo(0),

		autonTimer(),

		gyro(0),
		gyroPID(0,0,0,360,360),
		gyroPIDrunning(false),
		gyroPIDoutput(0.0),

		centerX(0),
		centerY(0)
	{
		cam = new AxisCamera("axis-camera.local");
	}

private:
	void RobotInit(){
		myRobot.SetSafetyEnabled(false);
		myRobot.SetExpiration(0.1); // watchdog is the WORST THING EVER

		gyro.Calibrate();

		cam->WriteBrightness(50);
		cam->WriteMaxFPS(15);
		cam->WriteResolution(AxisCamera::kResolution_320x240);
		//cam->WriteResolution( AxisCamera::kResolution_640x480);
		cam->WriteCompression(50);
		cam->GetImage();

		SmartDashboard::PutNumber("top power", 1.0);
		SmartDashboard::PutNumber("bottom power", 1.0);

		SmartDashboard::PutNumber("gyro P", 1);
		SmartDashboard::PutNumber("gyro I", 0);
		SmartDashboard::PutNumber("gyro D", 0);
	}

	void goalTracker() {
		ParticleAnalysisReport particle;
		// Threshold targetThreshold(70,140,22,255,100,255); // Sage, 2014
		Threshold targetThreshold(60,140,20,255,100,255);
		BinaryImage *goodTargetImage;


		targetImage=cam->GetImage();
		int particleXsum = 0;
		int particleYsum = 0;
		int totalParticleArea = 0;

		goodTargetImage=targetImage->ThresholdHSL(targetThreshold);
		for(int particleAccess=0;particleAccess<goodTargetImage->GetNumberParticles();particleAccess++)
		{
			particle=goodTargetImage->GetParticleAnalysisReport(particleAccess);

			//if (particle.particleArea > { // maybe later
			particleXsum += particle.center_mass_x * particle.particleArea;
			particleYsum += particle.center_mass_y * particle.particleArea;
			totalParticleArea += particle.particleArea;
		}

		centerX = double(particleXsum) / totalParticleArea;
		centerY = double(particleYsum) / totalParticleArea;
	}
	float getGoalAngleOffset() {
		/* TODO */
		// math your way from the x-val (from goalTracker) to an offset angle?

		// 47deg view angle
		// 320px wide
		return (centerX-160)*62.46/320; //either shift the 160 left or right or shift the whole angle (later maybe)
	}
	float getRequiredShotPower() {
		/* TODO
		 * Precondition: isValidShot(centerY)
		 */
		// math your way from the y-val (from goalTracker) to a required shot power?
		return 0.0;
	}
	bool isValidShot(float centerY) {
		/* TODO */
		// just check a y-value range
		return true;
	}
	void goToAngleWithPID(float angle) {
		/* TODO at competition */

		// gyro PID

		if(!gyroPIDrunning)
			gyroPID.ResetPID();

		gyroPID.Enable();
		gyroPID.SetPID(
			SmartDashboard::GetNumber("gyro P", 1),
			SmartDashboard::GetNumber("gyro I", 0),
			SmartDashboard::GetNumber("gyro D", 0)
		);

		gyroPID.SetEpsilon(0);
		gyroPID.SetMaxChangeSetpoint(0.01);

		gyroPID.SetSetpoint(angle);

		// might need to trueMap/constrain this
		gyroPIDoutput=gyroPID.GetOutput(gyro.GetAngle(),NULL,NULL,NULL);
		SmartDashboard::PutNumber("gyroPID Output",gyroPIDoutput);

		if(gyroPIDrunning)
			myRobot.ArcadeDrive(0, float(gyroPIDoutput));
		else
			myRobot.ArcadeDrive(0.0,0.0);
	}

	void AutonomousInit()
	{
		autonTimer.Reset();
		autonTimer.Start();

		// TODO: auton chooser?  Like from the sample
	}

	void AutonomousPeriodic()
	{
		if (autonTimer.Get() < 2)
			myRobot.ArcadeDrive(1.0, 0.0);
		else
			myRobot.ArcadeDrive(0.0, 0.0);
	}

	void TeleopInit() {}

	void TeleopPeriodic()
	{
		if(stick.GetRawButton(1)) {
			if (!gyroPIDrunning)
				goalTracker();

			SmartDashboard::PutNumber("x avg",centerX);
			SmartDashboard::PutNumber("y avg",centerY);
			SmartDashboard::PutNumber("angle offset", getGoalAngleOffset());

			//goToAngleWithPID(gyro.GetAngle() + getGoalAngleOffset());
			gyroPIDrunning = true;
		}
		else {
			myRobot.ArcadeDrive(-stick.GetY(), -stick.GetRawAxis(5));

			gyroPIDrunning = false;
			gyroPID.Disable();
		}

		if (stick.GetRawButton(4))
			sageArmSetVal = 0.45;
		else if (stick.GetRawButton(3))
			sageArmSetVal = -1.0;
		else
			sageArmSetVal = 0.0;

		winchSetVal = 0.0;
		whipSetVal = 0.0;
		if (stick.GetRawButton(9))
			winchSetVal = 1.0;
		else if (stick.GetRawButton(10))
			whipSetVal = 1.0;

		shooterSetVal = 0.0;
		intakeSetVal = 0.0;
		if(stick.GetRawButton(6))
			intakeSetVal = -0.5;

		if (stick.GetRawButton(2) && !longShootingNow) {
			longShootingNow = true;
			shooterTimer.Reset();
			shooterTimer.Start();

			topPower = SmartDashboard::GetNumber("top power", 1.0);
			bottomPower = SmartDashboard::GetNumber("bottom power", 1.0);
			// TODO: vision processing and things (maybe)
		}
		else if (stick.GetRawButton(5) && !shortShootingNow) {
			shortShootingNow = true;
			shooterTimer.Reset();
			shooterTimer.Start();
		}

		if (stick.GetRawButton(11)) {
			longShootingNow = false;
			shortShootingNow = false;
		}

		if (longShootingNow) {
			if (shooterTimer.Get() < 4) {
				shooterSetVal = topPower;
				intakeSetVal = -bottomPower;

				if (shooterTimer.Get() > 3)
					shootServo.Set(0.75); //0.5
			}
			else {
				shootServo.Set(0.5); //0.0

				shooterTimer.Stop();
				longShootingNow = false;
			}
		}
		else if (shortShootingNow) {
			if (shooterTimer.Get() < 1) {
				intakeSetVal = -0.5;
				shooterSetVal = -0.5;
				shootServo.Set(0.3);
			}
			if (shooterTimer.Get() > 2)
				intakeSetVal = 1.0;
			if (shooterTimer.Get() > 5)
				shootServo.Set(0.0);
			if (shooterTimer.Get() > 6) {
				shortShootingNow = false;
				shooterTimer.Stop();
			}
		}

		sageArm.Set(sageArmSetVal);
		winch1.Set(winchSetVal);
		winch2.Set(winchSetVal);
		whip.Set(whipSetVal);
		shooter.Set(shooterSetVal);
		intake.Set(intakeSetVal);

		SmartDashboard::PutNumber("Gyro", gyro.GetAngle());
	}

	void TestPeriodic()
	{
	}
};

START_ROBOT_CLASS(Robot)
