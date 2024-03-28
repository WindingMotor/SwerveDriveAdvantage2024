// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Auto;
import frc.robot.Constants.Auto.ScoringPoses;
import frc.robot.subsystems.swerve.SUB_Swerve;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;

public class CMD_AlignAuto extends Command {

	private final SUB_Swerve swerve;
	private final PIDController pid;

	private boolean isCommandDone;
	private int timer;

	public CMD_AlignAuto(SUB_Swerve swerve) {
		this.swerve = swerve;
		addRequirements(swerve);
		this.pid =
				new PIDController(
						Auto.SWERVE_ALIGN_PID.kP, Auto.SWERVE_ALIGN_PID.kI, Auto.SWERVE_ALIGN_PID.kD);
		this.pid.enableContinuousInput(-180, 180);
		this.pid.setTolerance(1.0);
	}

	// Print a message to the driver station and idle the robot subsystems
	@Override
	public void initialize() {
		isCommandDone = false;
		pid.reset();
	}

	@Override
	public void execute() {

		timer++;

		// Get allaince
		var alli = DriverStation.getAlliance();
		Pose2d targetSpeakerPose;

		// Assign target speaker pose based on the alliance
		if (alli.get() == Alliance.Blue) {
			targetSpeakerPose = ScoringPoses.BLU_SPEAKER.pose;
		} else if (alli.get() == Alliance.Red) {
			targetSpeakerPose = ScoringPoses.RED_SPEAKER.pose;
		} else {
			targetSpeakerPose = new Pose2d();
			DriverStation.reportError("[error] Could not find alliance for auto angle", false);
		}

		// Find X and H distances for trig on finding the angle to rotate to
		double xDistanceMeters = swerve.getPose().getX();
		double hDistanceMeters = PhotonUtils.getDistanceToPose(swerve.getPose(), targetSpeakerPose);

		// Find Y distance for applying top / botton angles of the speaker
		double yDistanceMeters = swerve.getPose().getY();

		double setpointRadians = Math.toRadians(180);

		// If the alliance is blue
		if (alli.get() == Alliance.Blue) {

			// If robot is ABOVE the amp with a middle tolerance of 0.5 meters
			if (yDistanceMeters > targetSpeakerPose.getY() + 0.5) {
				setpointRadians =
						Math.toRadians(90)
								- (Math.asin(xDistanceMeters / hDistanceMeters))
								+ Math.toRadians(180);

				// If robot is BELOW the amp with a middle tolerance of 0.5 meters
			} else if (yDistanceMeters < targetSpeakerPose.getY() - 0.5) {
				setpointRadians = (Math.toRadians(90) + (Math.asin(xDistanceMeters / hDistanceMeters)));
			}

			// If the alliance is red
		} else if (alli.get() == Alliance.Red) {

			// Apply offset to the xDistanceMeters beacuse the measurements are taken off the opposite
			// side of the field
			xDistanceMeters = ScoringPoses.RED_SPEAKER.pose.getX() - swerve.getPose().getX();

			// If robot is ABOVE the amp with a middle tolerance of 0.5 meters
			if (yDistanceMeters > targetSpeakerPose.getY() + 0.25) {
				setpointRadians = (Math.asin(xDistanceMeters / hDistanceMeters)) - Math.toRadians(270);

				// If robot is BELOW the amp with a middle tolerance of 0.5 meters
			} else if (yDistanceMeters < targetSpeakerPose.getY() - 0.25) {
				setpointRadians = (Math.asin(xDistanceMeters / hDistanceMeters)) + Math.toRadians(135);
			}
		}

		Logger.recordOutput("[CMD_Align] H Distance", hDistanceMeters);
		Logger.recordOutput("[CMD_Align] X Distance", xDistanceMeters);
		Logger.recordOutput("[CMD_Align] Calculated Angle", setpointRadians);
		Logger.recordOutput(
				"[CMD_Align] Desired Pose",
				new Pose2d(swerve.getPose().getTranslation(), new Rotation2d(setpointRadians)));

		Logger.recordOutput("[CMD_Align] Real Angle", swerve.getPose().getRotation().getDegrees());

		Logger.recordOutput(
				"[CMD_Align] Desired vs Real Diffrence",
				setpointRadians - swerve.getPose().getRotation().getDegrees());

		double output =
				pid.calculate(swerve.getPose().getRotation().getDegrees(), Math.toDegrees(setpointRadians));

		/*
		 * gyro based
		double output =
				pid.calculate(
						swerve.getYaw().getDegrees(), Math.toDegrees(setpointRadians));
		 */

		swerve.driveRaw(0.0, 0.0, output);

		// End command if the robot is aligned and within 3 degrees
		if (Math.abs(Math.toDegrees(setpointRadians) - swerve.getPose().getRotation().getDegrees())
				< 3) {
			isCommandDone = true;
		}
	}

	@Override
	public boolean isFinished() {
		return isCommandDone || timer > 400;
	}
}
