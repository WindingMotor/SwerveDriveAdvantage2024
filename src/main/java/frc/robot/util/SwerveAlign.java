// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.Supplier;

public class SwerveAlign {

	Supplier<Double> controllerInput;
	Supplier<Pose2d> robotPose;
	PIDController pid;

	private boolean isControllerInput;

	public SwerveAlign(Supplier<Double> controllerInput, Supplier<Pose2d> robotPose) {
		this.controllerInput = controllerInput;
		this.robotPose = robotPose;
		this.pid = new PIDController(0.01, 0.01, 0.001);
		//		this.pid = new PIDController(0.0002, 0.001, 0.0006);
		this.pid.enableContinuousInput(-Math.PI, Math.PI);
		isControllerInput = true;
	}

	public double get() {
		if (isControllerInput) {
			return controllerInput.get();
		} else {

			return controllerInput.get();

			/* MAYBE WORKS?
			double xDistanceMeters = robotPose.get().getX();
			double hDistanceMeters =
					PhotonUtils.getDistanceToPose(robotPose.get(), ScoringPoses.BLU_SPEAKER.pose);

			double yDistanceMeters = robotPose.get().getY();
			double setpointRadians = 0.0;
			if (yDistanceMeters > ScoringPoses.BLU_SPEAKER.pose.getY()) {
				setpointRadians = Math.toRadians(90) - (Math.asin(xDistanceMeters / hDistanceMeters));
			} else if (yDistanceMeters < ScoringPoses.BLU_SPEAKER.pose.getY()) {
				setpointRadians = Math.toRadians(90) + (Math.asin(xDistanceMeters / hDistanceMeters));
			}
			// 90 amp
			Logger.recordOutput("Output H Distance", hDistanceMeters);
			Logger.recordOutput("Output X Distance", xDistanceMeters);
			Logger.recordOutput("Output Angle", setpointRadians);
			Logger.recordOutput(
					"Auto Angle Pose",
					new Pose2d(robotPose.get().getTranslation(), new Rotation2d(setpointRadians)));
			return pid.calculate(robotPose.get().getRotation().getRadians(), setpointRadians);
			*/
		}
	}

	public void setControllerInput(boolean isControllerInput) {
		this.isControllerInput = isControllerInput;
	}
}
