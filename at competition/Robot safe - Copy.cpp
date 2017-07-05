#include <iostream>
#include <memory>
#include <string>

#include <WPILib.h>
#include <CANTalon.h>

#include <IterativeRobot.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>

#include <CameraServer.h>

#include <SageFunctions.h>
#include <PID.h>

#include <GripPipeline.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

class Robot: public frc::IterativeRobot {
public:
	Joystick stick;
	CANTalon left, right, climber, gearCatch;
	float climberSet, gearCatchSet;

	RobotDrive myRobot;
	bool inverseDrive, inverseDrivePressed;

	DigitalInput gearCatchLimit;
	bool gearCatchLimitOverride;

	Timer timz;

	AnalogGyro gyro;
	PID gyroPID;
	bool gyroPIDrunning;

	bool finalCountdown;

	// HSLImage *targetImage;
	cs::AxisCamera* cam;

	cs::UsbCamera springCam;
	cs::UsbCamera climberCam;
	//cs::CvSink camServer;

	double centerX, centerY;


	Robot():
		stick(0),
		left(11),
		right(10),
		climber(13),
		gearCatch(12),
		climberSet(0.0), gearCatchSet(0.0),
		myRobot(left, right),
		inverseDrive(false), inverseDrivePressed(false),

		gearCatchLimit(8), gearCatchLimitOverride(true),

		timz(),

		gyro(0),
		gyroPID(1, 0, 0, 360, 0),

		finalCountdown(false),

		centerX(0),
		centerY(0)
	{}

	void RobotInit() {
		SmartDashboard::PutBoolean("Gear Catch Limit Override", true); // TODO change for competition

//		SmartDashboard::PutNumber("gyro P", -20);
//		SmartDashboard::PutNumber("gyro I", 1000);
//		SmartDashboard::PutNumber("gyro D", 0);

		gyro.Calibrate();

		cam = new cs::AxisCamera("cam0", "axis-camera");

		springCam = CameraServer::GetInstance()->StartAutomaticCapture(0);
		climberCam = CameraServer::GetInstance()->StartAutomaticCapture(1);

		// excessively fancy chooser code
		//camServer = CameraServer::GetInstance()->GetVideo();
		//camServer.SetSource(springCam);
	}

	void gotoAngle(double targetAngle) {
		double currentAngle = gyro.GetAngle();
		while (fabs(currentAngle - targetAngle) > 180) {
			if (currentAngle > 0)
				currentAngle -= 360;
			else if (currentAngle < 0)
				currentAngle += 360;
		}

		if (!gyroPIDrunning) {
			gyroPID.ResetPID();
			gyroPID.Enable();
		}
		gyroPIDrunning = true;

		gyroPID.SetPID(-20, 1000, 0); // values calculated on 2017-03-20
		//gyroPID.SetPID(SmartDashboard::GetNumber("gyro P", 1), SmartDashboard::GetNumber("gyro I", 0), SmartDashboard::GetNumber("gyro D", 0));

		gyroPID.SetSetpoint(0.0);
		gyroPID.SetEpsilon(0);
		gyroPID.SetMaxChangeSetpoint(0.01);

		float output = gyroPID.GetOutput(currentAngle, NULL, NULL, NULL);

		myRobot.ArcadeDrive(0.0, output);
		SmartDashboard::PutNumber("output", output);
	}
	void springTracker() {
		// TODO
//		cs::VideoSink x = cam->EnumerateSinks()[0];
//		grip::GripPipeline().Process(x);

//		GripPipeline p;
//		cam->GetImage;
//		ParticleAnalysisReport particle;
//		// Threshold targetThreshold(70,140,22,255,100,255); // Sage, 2014
//		Threshold targetThreshold(60,140,20,255,100,255);
//		BinaryImage *goodTargetImage;
//
//
//		targetImage=cam->GetImage();
//		int particleXsum = 0;
//		int particleYsum = 0;
//		int totalParticleArea = 0;
//
//		goodTargetImage=targetImage->ThresholdHSL(targetThreshold);
//		for(int particleAccess=0;particleAccess<goodTargetImage->GetNumberParticles();particleAccess++)
//		{
//			particle=goodTargetImage->GetParticleAnalysisReport(particleAccess);
//
//			//if (particle.particleArea > { // maybe later
//			particleXsum += particle.center_mass_x * particle.particleArea;
//			particleYsum += particle.center_mass_y * particle.particleArea;
//			totalParticleArea += particle.particleArea;
//		}
//
//		centerX = double(particleXsum) / totalParticleArea;
//		centerY = double(particleYsum) / totalParticleArea;
	}

	void AutonomousInit() override {
		timz.Reset();
		timz.Start();
	}

	void AutonomousPeriodic() {
		if (timz.Get() < 3.25) {
			myRobot.ArcadeDrive(-0.6,0.0);
			SmartDashboard::PutNumber("auton stage", 1);
		} else if (timz.Get() < 3.75) {
			gotoAngle(0);
			SmartDashboard::PutNumber("auton stage", 2);
		} else {
			myRobot.ArcadeDrive(-0.4,0.0);
			SmartDashboard::PutNumber("auton stage", 4);
		}

		SmartDashboard::PutNumber("timz", timz.Get());
	}

	void TeleopInit() {
		timz.Reset();
		timz.Start();

		finalCountdown = false;
	}

	void TeleopPeriodic() {
		SmartDashboard::PutNumber("gyro", gyro.GetAngle());
		if (stick.GetRawButton(6)) {
			gyro.Reset();
		}

		// DRIVE
		if (stick.GetRawButton(5)) {
			gotoAngle(0);
		} else {
			gyroPID.Disable();
			gyroPIDrunning = false;

			float sensitivity = trueMap(-stick.GetRawAxis(2), 1.0, -1.0, 1.0, 0.5);
			myRobot.ArcadeDrive(stick.GetY()*sensitivity*(inverseDrive ? -1 : 1), -stick.GetRawAxis(5)*sensitivity);

			SmartDashboard::PutNumber("output", -stick.GetRawAxis(5)*sensitivity);
			//myRobot.ArcadeDrive(stick.GetY(),-stick.GetRawAxis(5)); //0);
		}

		if (stick.GetRawButton(10) && !inverseDrivePressed) {
			inverseDrive = !inverseDrive;
			inverseDrivePressed = true;
		} else if (!stick.GetRawButton(10)) {
			inverseDrivePressed = false;
		}

		// GEAR CATCH
		gearCatchSet = 0.0;

		SmartDashboard::PutBoolean("Gear Catch Limit", gearCatchLimit.Get());
		gearCatchLimitOverride = SmartDashboard::GetBoolean("Gear Catch Limit Override", true);

		if(stick.GetRawButton(3) && (!gearCatchLimit.Get() || gearCatchLimitOverride) ) {
			gearCatchSet = 1.0;
		}
		if(stick.GetRawButton(4)) {
			gearCatchSet = -1.0;
		}
		gearCatch.Set(gearCatchSet);

		// CLIMBER
		climberSet = 0.0;

		if(stick.GetRawButton(15)) {// if(stick.GetRawButton(1 && 15)) {
			climberSet = -1;
		}
		else if(stick.GetRawButton(1)) {
			climberSet = -.5;
		}
		climber.Set(climberSet);

		// CAMERA
		if (timz.Get() > 100) {
			//camServer.SetSource(climberCam);

			timz.Stop();
			timz.Reset();

			finalCountdown = true;
		}
		SmartDashboard::PutBoolean("IT'S THE FINAL COUNTDOWN!", finalCountdown);
		SmartDashboard::PutNumber("timz", timz.Get());

	}

	// TODO: easy gear drop
	void TestInit() {

	}
	void TestPeriodic() {
	}

};

START_ROBOT_CLASS(Robot)
