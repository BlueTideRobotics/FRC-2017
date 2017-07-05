#define AUTON_DRIVE_HIGH 0.6
#define AUTON_DRIVE_LOW 0.5

#define FORWARD_SIGN -1
//#define DRIFT_CORRECT -0.26
#define DRIFT_CORRECT 0.0

#define CHUTE_ANGLE 20
#define SPRING_ANGLE 25

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

//#include <CameraServer.h>

#include <SageFunctions.h>
#include <PID.h>

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

	int autonomousMode;

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

		autonomousMode(1)

	{

	}

	void RobotInit() {
		SmartDashboard::PutBoolean("Gear Catch Limit Override", false);
		SmartDashboard::PutNumber("Autonomous mode", 1);
//
//		SmartDashboard::PutNumber("gyro P", -20);
//		SmartDashboard::PutNumber("gyro I", 1000);
//		SmartDashboard::PutNumber("gyro D", 0);
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

			//gyroPID.SetPID(-20, 1000, 0); // values calculated on 2017-03-20 with practice robot
			gyroPID.SetPID(-50, 5000, 0); // values calculated at competition
			//gyroPID.SetPID(SmartDashboard::GetNumber("gyro P", -20), SmartDashboard::GetNumber("gyro I", 5000), SmartDashboard::GetNumber("gyro D", 0));
			gyroPID.SetEpsilon(0);
			gyroPID.SetMaxChangeSetpoint(0.01);
		}
		gyroPIDrunning = true;
		gyroPID.SetSetpoint(targetAngle);

		float output = gyroPID.GetOutput(currentAngle, NULL, NULL, NULL);

		SmartDashboard::PutNumber("PID output", output);
		myRobot.ArcadeDrive(forwardPower, output);
	}
	void disablePID() {
		gyroPID.Disable(); gyroPIDrunning = false;
	}

	void AutonomousInit() override {
		timz.Reset();
		timz.Start();

		gyro.Reset();

		autonomousMode = SmartDashboard::GetNumber("Autonomous mode", 1);
		//myRobot.SetSafetyEnabled(false);
	}
	void AutonomousPeriodic() override {
		SmartDashboard::PutNumber("timz", timz.Get());
		SmartDashboard::PutNumber("gyro", gyro.GetAngle());
//		myRobot.ArcadeDrive(0.6, -0.15);

		if (autonomousMode == 0) {
			if (timz.Get() < 5) {
				myRobot.ArcadeDrive(AUTON_DRIVE_HIGH * FORWARD_SIGN, DRIFT_CORRECT);
			} else {
				myRobot.ArcadeDrive(0.0, 0.0);
			}
		} else if (autonomousMode == 1) {
			if (timz.Get() < 5) {
				gotoAngle(0.0, AUTON_DRIVE_HIGH * FORWARD_SIGN, false);
			} else {
				myRobot.ArcadeDrive(0.0, 0.0);
			}
		} else if (autonomousMode == 2 || autonomousMode == 3) {
			if (timz.Get() < 3) {
				gotoAngle(0.0, AUTON_DRIVE_HIGH * FORWARD_SIGN, false);
			} else if (timz.Get() < 4) {
				gotoAngle(SPRING_ANGLE*(autonomousMode == 2 ? 1 : -1), 0.0, false);
			} else if (timz.Get() < 7){
				gotoAngle(SPRING_ANGLE*(autonomousMode == 2 ? 1 : -1), AUTON_DRIVE_HIGH * FORWARD_SIGN, false);
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


		if (stick.GetRawButton(5)){
			gotoAngle(0, 0, true);
		} else if (stick.GetRawButton(3)){
			gotoAngle(CHUTE_ANGLE, 0, true);
		} else if (stick.GetRawButton(4)){
			gotoAngle(-CHUTE_ANGLE, 0, true);
		} else {
			disablePID();
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
	}

	void TestInit() {

	}
	void TestPeriodic() {
	}

};

START_ROBOT_CLASS(Robot)
