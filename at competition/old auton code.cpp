// one adjustment w/ gyro only
			if (timz.Get() < 2.75) {
				myRobot.ArcadeDrive(-AUTON_HIGH_POWER,0.0);
			} else if (timz.Get() < 3.25) {
				gotoAngle(0);
			} else {
				gyroPID.Disable();
				gyroPIDrunning = false;

				myRobot.ArcadeDrive(-AUTON_STEP_POWER,0.0);
			}
			// one adjustment w/ vision
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