#define AUTONOMOUS_MODE 1
#define AUTON_LOW_POWER 0.4
#define AUTON_HIGH_POWER 0.6

#define AUTON_STEP_TIME 1.0
#define AUTON_DRIVE_FRAC 0.5 /* fraction of each step spent driving */

#include <iostream>
#include <memory>
#include <string>

#include <WPILib.h>
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
	Talon left, right, climber, gearCatch;
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

	cs::AxisCamera cam;

	cs::UsbCamera springCam;
	cs::UsbCamera climberCam;
	//cs::CvSink camServer;

	cs::CvSink cvSink;
	cv::Mat mat;
	grip::GripPipeline imgProcessor;

	double visionAngle;
	int autonIters;
	double autonTargetAngle;
	bool visionSuccess;
	double nextAutonStepTime;

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

		cam("cam0", "axis-camera"),
		mat(),
		imgProcessor(),

		visionAngle(0),

		autonIters(0),
		autonTargetAngle(0),
		visionSuccess(false),
		nextAutonStepTime(0.0) // TODO

	{
		cam = CameraServer::GetInstance()->AddAxisCamera("axis-camera");
		//cam.SetResolution(640, 480);

		springCam = CameraServer::GetInstance()->StartAutomaticCapture(0);
		climberCam = CameraServer::GetInstance()->StartAutomaticCapture(1);

		cvSink = CameraServer::GetInstance()->GetVideo();
	}

	void RobotInit() {
		SmartDashboard::PutBoolean("Gear Catch Limit Override", true); // TODO change for competition

//		SmartDashboard::PutNumber("gyro P", -20);
//		SmartDashboard::PutNumber("gyro I", 1000);
//		SmartDashboard::PutNumber("gyro D", 0);
		SmartDashboard::PutNumber("visionGain", 1.2);
		SmartDashboard::PutNumber("gyroGain", -0.7);
		SmartDashboard::PutNumber("combinedGain", 1.1);

		gyro.Calibrate();

		// excessively fancy camera chooser code
		//camServer = CameraServer::GetInstance()->GetVideo();
		//camServer.SetSource(springCam);
	}

	/*
	 * Uses gyro PID to twist robot to given angle.
	 * Run this when you're done:
	 * 		gyroPID.Disable();
	 *		gyroPIDrunning = false;
	 */
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
		SmartDashboard::PutNumber("gyro PID output", output);
	}

	/*
	 * Uses AXIS camera to accurately set `visionAngle`
	 * If there's an error, returns `false` and leaves `visionAngle` untouched
	 * If no errors, returns `true` and sets `visionAngle` appropriately
	 */
	bool updateVisionAngle() {
		if (cvSink.GrabFrame(mat) == 0) {
			return false;
		}

		imgProcessor.Process(mat);
		cv::Mat* adjusted = imgProcessor.GetCvErodeOutput();

		if (adjusted->empty()) {
			return false;
		} else {
			float sumX = 0;
			float numPixels = 0;
			for(int x = 0; x<adjusted->cols; x++) {
				for(int y = 0; y<adjusted->rows; y++) {
					if(adjusted->at<uchar>(y,x) >= 50) {
						sumX += x;
						numPixels++;
					}
				}
			}
			// adapted from http://answers.opencv.org/question/460/finding-centroid-of-a-mask/

			double centerX = double(sumX) / numPixels;

			// 47deg view angle
			// 320px wide
			visionAngle = (centerX-160)*62.46/320; //either shift the 160 left or right or shift the whole angle (later maybe)

			return true;
		}
	}

	double autoAlignFormula(double gyroAngle, double visionAngle) {
		return (visionAngle*SmartDashboard::GetNumber("visionGain", 1.2) - gyroAngle*SmartDashboard::GetNumber("gyroGain", -0.7)) * SmartDashboard::GetNumber("combinedGain", 1.1);
	}

	void AutonomousInit() override {
		timz.Reset();
		timz.Start();
	}
	void AutonomousPeriodic() {
		/* TODO:
		 * 0. Just go straight
		 * 1. Go straight, gyro correct
		 * 2. Go straight, get heading from camera, use gyro to correct
		 * 3. Bushroe algorithm
		 */
		switch(AUTONOMOUS_MODE) { // TODO: add checks for if vision experiences an epic failx
		case 0:
			if (timz.Get() < 4) {
				myRobot.ArcadeDrive(-AUTON_LOW_POWER,0.0);
			} else {
				myRobot.ArcadeDrive(0.0,0.0);
			}
			break;
		case 1:
			if (timz.Get() < 3.25) {
				myRobot.ArcadeDrive(-AUTON_HIGH_POWER,0.0);
			} else if (timz.Get() < 3.75) {
				gotoAngle(0);
			} else {
				gyroPID.Disable();
				gyroPIDrunning = false;

				myRobot.ArcadeDrive(-AUTON_LOW_POWER,0.0);
			}
			break;
		case 2:
			if (timz.Get() < 3.25) {
				myRobot.ArcadeDrive(-AUTON_HIGH_POWER,0.0);
			} else if (timz.Get() < 3.75) {
				if (visionSuccess) {
					gotoAngle(autonTargetAngle);
				} else if (updateVisionAngle()) {
					visionSuccess = true;
					autonTargetAngle = gyro.GetAngle() + visionAngle;

					myRobot.ArcadeDrive(0.0,0.0);
				}
			} else if (timz.Get() < 5) {
				gyroPID.Disable();
				gyroPIDrunning = false;

				myRobot.ArcadeDrive(-AUTON_LOW_POWER, 0.0);
			}
			break;
		case 3:
			if (timz.Get() < 6) {
				if (timz.Get() >= nextAutonStepTime - (AUTON_STEP_TIME*AUTON_DRIVE_FRAC)) {
					myRobot.ArcadeDrive(-AUTON_LOW_POWER, 0.0);
					gyroPID.Disable();
					gyroPIDrunning = false;
				}
				else if (timz.Get() >= nextAutonStepTime) {
					if (visionSuccess) {
						nextAutonStepTime += AUTON_STEP_TIME;
					} else if (updateVisionAngle()) {
						visionSuccess = true;
						autonTargetAngle = autoAlignFormula(gyro.GetAngle(), visionAngle); //gyro.GetAngle() + visionAngle;
					}
					myRobot.ArcadeDrive(0.0,0.0);
				} else {
					gotoAngle(autonTargetAngle);
					visionSuccess = false;
				}
			} else {
				gyroPID.Disable();
				gyroPIDrunning = false;

				myRobot.ArcadeDrive(0.0, 0.0);
			}
			break;
			//nextAutonStepTime
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
