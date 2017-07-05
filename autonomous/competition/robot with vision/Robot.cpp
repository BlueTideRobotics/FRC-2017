#define AUTON_DRIVE_HIGH 0.6
#define AUTON_DRIVE_LOW 0.5

#define FORWARD_SIGN -1
//#define DRIFT_CORRECT -0.26
#define DRIFT_CORRECT 0.0

#define CHUTE_ANGLE 20

#define ALIGN_STRAIGHT_TIME 0.1
#define ALIGN_TURN_TIME 0.2
#define ALIGN_VISION_TIME 0.1


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
	bool gearCatchDirection, gearCatchTogglePressed;

	RobotDrive myRobot;
	bool inverseDrive, inverseDrivePressed;

	DigitalInput gearCatchDown;
	bool gearCatchLimitOverride;

	Timer timz;

	AnalogGyro gyro;
	PID gyroPID;
	bool gyroPIDrunning;

	cs::AxisCamera cam;

//	cs::UsbCamera springCam;
//	cs::UsbCamera climberCam;
	//cs::CvSink camServer;

	cs::CvSink cvSink;
	cv::Mat mat;
	grip::GripPipeline imgProcessor;

	int autonomousMode;
	bool autoAlignRunning;

	float visionAngle, alignTargetAngle;
	int numPixels;

	float t_alignStart, t_straight, t_turn, t_camera;
	int alignmentStage;

	bool teleopVisionSuccess;

	Robot():
		stick(0),
		left(11),
		right(10),
		climber(13),
		gearCatch(12),
		climberSet(0.0), gearCatchSet(0.0),

		gearCatchDirection(false),
		gearCatchTogglePressed(false),

		myRobot(left, right),
		inverseDrive(false), inverseDrivePressed(false),

		gearCatchDown(8), gearCatchLimitOverride(false),

		timz(),

		gyro(0),
		gyroPID(1, 0, 0, 360, 0),
		gyroPIDrunning(false),

		cam("cam0", "10.28.40.21"), //"axis-camera"),
		mat(),
		imgProcessor(),

		autonomousMode(1),
		autoAlignRunning(false),

		visionAngle(0), alignTargetAngle(0),
		numPixels(0),

		t_alignStart(0), t_straight(0), t_turn(0), t_camera(0),
		alignmentStage(0), teleopVisionSuccess(false)

	{
		cam = CameraServer::GetInstance()->AddAxisCamera("10.28.40.21"); //"169.254.177.148"); //("10.28.40.21"); //("axis-camera");
		//cam.SetResolution(640, 480);
		cvSink = CameraServer::GetInstance()->GetVideo();
	}

	void RobotInit() {
		SmartDashboard::PutBoolean("Gear Catch Limit Override", false);
		SmartDashboard::PutNumber("Autonomous mode", 0);
//
//		SmartDashboard::PutNumber("gyro P", -20);
//		SmartDashboard::PutNumber("gyro I", 1000);
//		SmartDashboard::PutNumber("gyro D", 0);

//		SmartDashboard::PutNumber("visionGain", 1.5);
//		SmartDashboard::PutNumber("gyroGain", -0.7);
//		SmartDashboard::PutNumber("combinedGain", 1.3);

		SmartDashboard::PutNumber("init gain", 1.75);
		SmartDashboard::PutNumber("late gain", 1.2);
		SmartDashboard::PutNumber("gain change cutoff", 2.5);
		SmartDashboard::PutNumber("align angle max", 35);

		//CameraServer::GetInstance()->StartAutomaticCapture(2);

		gyro.Calibrate();

		// excessively fancy camera chooser code
		//camServer = CameraServer::GetInstance()->GetVideo();
		//camServer.SetSource(springCam);
	}

	/*
	 * Uses gyro PID to twist robot to given angle.
	 * Call disablePID() when you're done
	 */
	void gotoAngle(double targetAngle, double forwardPower = 0.0, bool nearest = false) {
		float currentAngle = gyro.GetAngle();
		while (nearest && fabs(currentAngle - targetAngle) > 180) {
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

		//gyroPID.SetPID(-20, 1000, 0); // values calculated on 2017-03-20 with practice robot
		gyroPID.SetPID(-20, 5000, 0); // values calculated at competition
		//gyroPID.SetPID(SmartDashboard::GetNumber("gyro P", -20), SmartDashboard::GetNumber("gyro I", 1000), SmartDashboard::GetNumber("gyro D", 0));

		gyroPID.SetSetpoint(targetAngle);
		gyroPID.SetEpsilon(0);
		gyroPID.SetMaxChangeSetpoint(0.01);

		float output = gyroPID.GetOutput(currentAngle, NULL, NULL, NULL);

		SmartDashboard::PutNumber("PID output", output);
		myRobot.ArcadeDrive(forwardPower, output);
	}
	void disablePID() {
		gyroPID.Disable(); gyroPIDrunning = false;
	}
	/*
	 * Uses AXIS camera to accurately set `visionAngle`
	 * If there's an error, returns `false` and leaves `visionAngle` untouched
	 * If no errors, returns `true` and sets `visionAngle` appropriately
	 */
	bool updateVisionAngle() {
		/* testing code
		visionAngle = SmartDashboard::GetNumber("fake vision angle", 5);
		return true;
		 </testing code> */

		if (cvSink.GrabFrame(mat) == 0) {
			SmartDashboard::PutNumber("vision fail", 1);
			return false;
		}

		imgProcessor.Process(mat);
		cv::Mat* adjusted = imgProcessor.GetHslThresholdOutput();

		double centerX;

//		if (imgProcessor.GetFilterContoursOutput()->empty()) {
//			return false;
//		} else {
//			double sumX = 0;
//			int numPixels = 0;
//			for(int i = 0; i < min(2, imgProcessor.GetFilterContoursOutput()->size()); i++) {
//				cv::Rect r = cv::boundingRect(imgProcessor.GetFilterContoursOutput()->at(i));
//				double area = cv::contourArea(imgProcessor.GetFilterContoursOutput()->at(i));
//				sumX += area * (r.x + (r.width / 2.0));
//				numPixels += area;
		if (adjusted->empty()) {
			SmartDashboard::PutNumber("vision fail", 2);
			return false;
		} else {
			float sumX = 0;
			float numPixels = 0;
			for(int x = 0; x<adjusted->cols; x++) {
				for(int y = adjusted->rows / 2 ; y<adjusted->rows; y++) {
					if(adjusted->at<uchar>(y,x)) {
						sumX += x; // * mat.at<uchar>(y,x);
						numPixels++; // += mat.at<uchar>(y,x);
					}
				}
			}
			// adapted from http://answers.opencv.org/question/460/finding-centroid-of-a-mask/
			//SmartDashboard::PutNumber("numPixels", numPixels);

			if (numPixels == 0) { //if (numPixels < 30) {
				SmartDashboard::PutNumber("vision fail", 3);
				return false;
			}
			centerX = sumX / numPixels;

//			cv::Moments m = cv::moments(* imgProcessor.GetFilterContoursOutput()); //cv::moments(mat);
//			double centerX = m.m10 / m.m00;

			//SmartDashboard::PutNumber("centerX", centerX);
			int width = mat.cols;
			// 47deg view angle
			// 320px wide
			visionAngle = (centerX-(width/2.0))*(47.0/width); // *62.46/320; //either shift the 160 left or right or shift the whole angle (later maybe)

			return true;
		}
	}

	double __autoAlignFormula(double gyroAngle, double visionAngle) {
		return (visionAngle*1.3 + gyroAngle*0.7) * (timz.Get() - t_alignStart < SmartDashboard::PutNumber("gain change cutoff", 2.5) ? SmartDashboard::GetNumber("init gain", 1.6) : SmartDashboard::GetNumber("late gain", 1.2));
	}
	void autoAlign(float forwardPower=FORWARD_SIGN * AUTON_DRIVE_HIGH) {
		if (!autoAlignRunning) {
			alignTargetAngle = 0;

			alignmentStage = 0;
			t_straight = timz.Get();
			t_alignStart = timz.Get();

			autoAlignRunning = true;
		}

		switch(alignmentStage) {
		case 0:
			if (timz.Get() > t_straight + ALIGN_STRAIGHT_TIME) {
				alignmentStage = 1;
				t_camera = timz.Get();
			}
			myRobot.ArcadeDrive(forwardPower, 0.0);
			break;
		case 1:
			myRobot.ArcadeDrive(forwardPower, 0.0);
			if (updateVisionAngle() || timz.Get() > t_camera + ALIGN_VISION_TIME) {
				alignTargetAngle = __autoAlignFormula(gyro.GetAngle(), visionAngle);
				t_turn = timz.Get();
				alignmentStage = 2;

				disablePID();
			}
			break;
		case 2:
			if (timz.Get() > t_turn + ALIGN_TURN_TIME) {
				alignmentStage = 0;
				t_straight = timz.Get();
				myRobot.ArcadeDrive(0.0, 0.0);
			} else {
				gotoAngle(fabs(alignTargetAngle) > SmartDashboard::GetNumber("align angle max", 35) ? 0 : alignTargetAngle, forwardPower);
			}
			break;
		default: // shouldn't ever happen but whatever
			myRobot.ArcadeDrive(0.0, 0.0);
		}
	}
	void autoAlignDisable() {
		autoAlignRunning = false;
	}

	void AutonomousInit() override {
		timz.Reset();
		timz.Start();

		gyro.Reset();

		autonomousMode = SmartDashboard::GetNumber("Autonomous mode", 0);
		//myRobot.SetSafetyEnabled(false);
	}
	void AutonomousPeriodic() override {
		SmartDashboard::PutNumber("timz", timz.Get());
		SmartDashboard::PutNumber("target angle", alignTargetAngle);
		SmartDashboard::PutNumber("visionAngle", visionAngle);
		SmartDashboard::PutNumber("gyro", gyro.GetAngle());
//		myRobot.ArcadeDrive(0.6, -0.15);

		if (autonomousMode == 0) {
			if (timz.Get() < 7) {
				myRobot.ArcadeDrive(AUTON_DRIVE_HIGH * FORWARD_SIGN, DRIFT_CORRECT);
			} else {
				myRobot.ArcadeDrive(0.0, 0.0);
			}
		} else if (autonomousMode == 1) {
			if (timz.Get() < 5) {
				autoAlign(AUTON_DRIVE_HIGH * FORWARD_SIGN);
			} else if (timz.Get() < 8) {
				gotoAngle(0, AUTON_DRIVE_HIGH * FORWARD_SIGN, false);
			}
			else {
				autoAlignDisable();
				myRobot.ArcadeDrive(-AUTON_DRIVE_LOW, 0.0);
			}
		} else if (autonomousMode == 2 || autonomousMode == 3){
			if  (timz.Get() < 2) {
				myRobot.ArcadeDrive(-AUTON_DRIVE_HIGH, 0.0);
			} else if (timz.Get() < 5) {
				//updateVisionAngle();
				//&& (numPixels > 25 || fabs(gyro.GetAngle() > 10))

				gotoAngle(autonomousMode == 2 ? 15 : -15); //60 : -60);
			} else if (timz.Get() < 12) {
				autoAlign(-AUTON_DRIVE_HIGH);
			} else {
				autoAlignDisable();
				myRobot.ArcadeDrive(-AUTON_DRIVE_LOW, 0.0);
			}
		}
	}


	void TeleopInit() {
		timz.Reset();
		timz.Start();
	}

	void TeleopPeriodic() {
		SmartDashboard::PutNumber("gyro", gyro.GetAngle());

		// DRIVE
		float sensitivity = trueMap(-stick.GetRawAxis(2), 1.0, -1.0, 1.0, 0.5);

		float forwardPower = stick.GetY()*sensitivity*(inverseDrive ? -1 : 1);
		forwardPower = deadZone(forwardPower, 0.05);

		float twist = -stick.GetRawAxis(5)* sensitivity;
//		if (forwardPower > 0) {
////			if (twist > 0) {
////				twist = constrain(twist * 5, 1.0, -1.0);
////			}
//			twist = constrain(twist - DRIFT_CORRECT, 1.0, -1.0); // twist - DRIFT_CORRECT;
//		}
//		SmartDashboard::PutBoolean("drift correct active", forwardPower > 0);

		// .26
		SmartDashboard::PutNumber("left current", left.GetOutputCurrent());
		SmartDashboard::PutNumber("left out", left.Get());
		SmartDashboard::PutNumber("right current", left.GetOutputCurrent());
		SmartDashboard::PutNumber("right out", left.Get());


		if (stick.GetRawButton(5)) {
//			gotoAngle(SmartDashboard::GetNumber("target angle", 0), stick.GetY());
			autoAlign(forwardPower);
		} else if (stick.GetRawButton(6)){
			gotoAngle(0, 0, true);
		} else if (stick.GetRawButton(3)){
			gotoAngle(CHUTE_ANGLE, 0, true);
		} else if (stick.GetRawButton(4)){
			gotoAngle(-CHUTE_ANGLE, 0, true);
		} else {
//			disablePID();
			autoAlignDisable();

			myRobot.ArcadeDrive(forwardPower, twist);
		}

		if (stick.GetRawButton(10) && !inverseDrivePressed) {
			inverseDrive = !inverseDrive;
			inverseDrivePressed = true;
		} else if (!stick.GetRawButton(10)) {
			inverseDrivePressed = false;
		}

		// GEAR CATCH
		gearCatchSet = 0.0;

		SmartDashboard::PutBoolean("Gear Catch Limit", gearCatchDown.Get());
		gearCatchLimitOverride = SmartDashboard::GetBoolean("Gear Catch Limit Override", false);

//		if(stick.GetRawButton(3) && (!gearCatchLimit.Get() || gearCatchLimitOverride) ) {
//			gearCatchSet = 1.0;
//		}
//		if(stick.GetRawButton(4)) {
//			gearCatchSet = -1.0;
//		}
		if (stick.GetRawButton(7)) {
			if (!gearCatchTogglePressed) {
				gearCatchDirection = !gearCatchDirection;
				gearCatchTogglePressed = true;
			}
			if (gearCatchDirection && (gearCatchDown.Get() || gearCatchLimitOverride)) {
				gearCatchSet = 1.0;
			} else if (!gearCatchDirection) {
				gearCatchSet = -1.0;
			}
		} else {
			gearCatchTogglePressed = false;
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

		SmartDashboard::PutBoolean("IT'S THE FINAL COUNTDOWN!", timz.Get() > 100);
		SmartDashboard::PutBoolean("IT'S THE PENULTIMATE COUNTDOWN!", timz.Get() > 90);
		SmartDashboard::PutNumber("timz", timz.Get());


		if (stick.GetRawButton(11)) {
			gyro.Calibrate();
		}
		if (stick.GetRawButton(12)) {
			gyro.Reset();
		}

		if (stick.GetRawButton(2) && !teleopVisionSuccess) {
			teleopVisionSuccess = updateVisionAngle();
		} else if (!stick.GetRawButton(2)) {
			teleopVisionSuccess = false;
		}
		SmartDashboard::PutNumber("visionAngle", visionAngle);
		SmartDashboard::PutBoolean("teleopVisionSuccess", teleopVisionSuccess);
	}

	void TestInit() {

	}
	void TestPeriodic() {
	}

};

START_ROBOT_CLASS(Robot)
