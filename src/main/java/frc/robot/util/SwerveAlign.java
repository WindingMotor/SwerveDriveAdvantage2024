// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.Auto;
import frc.robot.Constants.Auto.ScoringPoses;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;

public class SwerveAlign {

	Supplier<Double> controllerInput;
	Supplier<Pose2d> robotPose;
	PIDController pid;

	private boolean isControllerInput;

	public SwerveAlign(Supplier<Double> controllerInput, Supplier<Pose2d> robotPose) {
		this.controllerInput = controllerInput;
		this.robotPose = robotPose;
		this.pid =
				new PIDController(
						Auto.SWERVE_ALIGN_PID.kP, Auto.SWERVE_ALIGN_PID.kI, Auto.SWERVE_ALIGN_PID.kD);
		this.pid.enableContinuousInput(-Math.PI, Math.PI);
		isControllerInput = true;
	}

	public double get() {
		if (isControllerInput) {
			return controllerInput.get();
		} else {

			var alli = DriverStation.getAlliance();
			Pose2d targetSpeakerPose;

			if (alli.get() == Alliance.Blue) {
				targetSpeakerPose = ScoringPoses.BLU_SPEAKER.pose;
			} else if (alli.get() == Alliance.Red) {
				targetSpeakerPose = ScoringPoses.RED_SPEAKER.pose;
			} else {
				targetSpeakerPose = new Pose2d();
				DriverStation.reportError("[error] Could not find alliance for auto angle", false);
			}

			/* Works quite well, maybe a bit more tuning though? */
			double xDistanceMeters = robotPose.get().getX();
			double hDistanceMeters = PhotonUtils.getDistanceToPose(robotPose.get(), targetSpeakerPose);

			double yDistanceMeters = robotPose.get().getY();
			double setpointRadians = 0.0;

			if (alli.get() == Alliance.Blue) {
				if (yDistanceMeters > targetSpeakerPose.getY() + 0.5) {
					setpointRadians = Math.toRadians(90) - (Math.asin(xDistanceMeters / hDistanceMeters));

				} else if (yDistanceMeters < targetSpeakerPose.getY() - 0.5) {
					setpointRadians =
							(Math.toRadians(90) + (Math.asin(xDistanceMeters / hDistanceMeters)))
									+ Math.toRadians(180);
				}
			} else if (alli.get() == Alliance.Red) {

				xDistanceMeters = ScoringPoses.RED_SPEAKER.pose.getX() - robotPose.get().getX();

				if (yDistanceMeters > targetSpeakerPose.getY() + 0.5) {
					setpointRadians = (Math.asin(xDistanceMeters / hDistanceMeters)) - Math.toRadians(270);

				} else if (yDistanceMeters < targetSpeakerPose.getY() - 0.5) {
					setpointRadians = (Math.asin(xDistanceMeters / hDistanceMeters)) + Math.toRadians(135);
				}
			}

			Logger.recordOutput("Output H Distance", hDistanceMeters);
			Logger.recordOutput("Output X Distance", xDistanceMeters);
			Logger.recordOutput("Output Angle", setpointRadians);
			Logger.recordOutput(
					"Auto Angle Pose",
					new Pose2d(robotPose.get().getTranslation(), new Rotation2d(setpointRadians)));

			return pid.calculate(robotPose.get().getRotation().getRadians(), setpointRadians);
		}
	}

	public void setControllerInput(boolean isControllerInput) {
		this.isControllerInput = isControllerInput;
	}
}
