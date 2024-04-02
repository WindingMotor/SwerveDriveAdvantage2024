// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.Auto.ScoringPoses;
import frc.robot.Constants.RobotMode;
import frc.robot.Constants.RotationOverrideState;
import frc.robot.subsystems.vision.SUB_Vision;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;

public class SUB_Swerve extends SubsystemBase {

	// old old 0.041215128954433
	// newer old  0.0429348178661

	private final IO_SwerveBase io;

	private final SwerveInputsAutoLogged inputs = new SwerveInputsAutoLogged();

	private final SUB_Vision vision;

	// Odometry lock, prevents updates while reading data
	private static final Lock odometryLock = new ReentrantLock();

	/*
	 *
	 * Old VAGLE PID Values
	 * new PIDConstants(5.0, 0.0, 0.0, 0.0), // Translation
	 * new PIDConstants(7.0, 0.0, 0.0, 0.0), // Heading
	 *
	 * Team 401 PID values, seem to work smoother at higher speeds
	 * TRANSLATION 2, 0, 0
	 * ROTATION 7.0, 4.5, 0
	 *
	 */
	public SUB_Swerve(IO_SwerveBase io, SUB_Vision vision, CommandXboxController driverController) {
		this.io = io;
		this.vision = vision;

		// 						new PIDConstants(13.0, 5.0, 0.0, 0.0), // Heading
		AutoBuilder.configureHolonomic(
				io::getPose, // Gets current robot pose
				io::resetOdometry, // Resets robot odometry if path has starter pose
				io::getRobotVelocity, // Gets chassis speed in robot relative
				io::setChassisSpeeds, // Drives the robot in robot relative chassis speeds
				new HolonomicPathFollowerConfig(
						new PIDConstants(5.0, 0.0, 0.0, 0.0), // Translation
						io.getHeadingPID(),
						/*
						 * IMPORTANT NOTE: These auto PIDs only have a relatively small effect. For a larger and better controlled one use the YAGSL PIDF config.
						 * When setting these PID values make sure rotation is faster than translation and there is a feedforward on the YAGSL PIDF config.
						 */
						Constants.Maestro.MAX_MODULE_SPEED_MPS,
						io.getConfigurationRadius(), // Drive base radius in meters
						new ReplanningConfig() // Replanning config see docs
						),
				() -> {
					// Auto path flipper for alliance color, always make paths on blue side
					var alliance = DriverStation.getAlliance();
					return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
				},
				this);

		PPHolonomicDriveController.setRotationTargetOverride(this::getRotationOverride);

	}

	public void periodic() {

		odometryLock.lock();
		io.updateInputs(inputs);
		if (Constants.CURRENT_MODE != RobotMode.SIM) {
			io.updateEstimations(vision);
		}
		io.updateOdometry();
		odometryLock.unlock();

		Logger.processInputs("Swerve", inputs);
	}

	/**
	 * Drives the robot, in field-relative, based of the specified inputs.
	 *
	 * @param translationX A supplier for the X translation
	 * @param translationY A supplier for the Y translation
	 * @param angularRotationX A supplier for the angular rotation
	 * @return The command for driving the swerve
	 */
	public Command drive(
			DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {

		return run(
				() -> {
					var alli = DriverStation.getAlliance();

					if (alli.get() == Alliance.Blue) {

						io.drive(

								// BLU

								new Translation2d(
										translationX.getAsDouble() * io.getMaximumVelocity(),
										-translationY.getAsDouble() * io.getMaximumVelocity()),
								-angularRotationX.getAsDouble() * io.getMaximumAngularVelocity(),
								true,
								true);

					} else if (alli.get() == Alliance.Red) {

						io.drive(
								// RED
								new Translation2d(
										-translationX.getAsDouble() * io.getMaximumVelocity(),
										translationY.getAsDouble() * io.getMaximumVelocity()),
								-angularRotationX.getAsDouble() * io.getMaximumAngularVelocity(),
								true,
								true);

					} else {
						DriverStation.reportError("DRIVE COULD NOT FIND ALLIANCE", false);

						// Default to blue
						io.drive(
								new Translation2d(
										translationX.getAsDouble() * io.getMaximumVelocity(),
										-translationY.getAsDouble() * io.getMaximumVelocity()),
								-angularRotationX.getAsDouble() * io.getMaximumAngularVelocity(),
								true,
								true);
					}
				});
	}

	/**
	 * Drives the robot to the specified pose.
	 *
	 * @param pose The target pose to drive to
	 * @return The command to run to drive to the pose.
	 */
	public Command driveToPose(Pose2d pose) {
		Command pathfindingCommand =
				AutoBuilder.pathfindToPose(
						pose,
						new PathConstraints(
								4.0, // autos in PathPlanner is 5.5
								3.0, // autos in PathPlanner is 3.3
								Units.degreesToRadians(540), // 540 350
								Units.degreesToRadians(720) // 720 // 540
								));
		return pathfindingCommand;
	}

	/**
	 * Drives the robot to the speaker depending on the alliance.
	 *
	 * @return The command to run to drive to the pose.
	 */
	public Command driveToSpeaker() {

		return new PrintCommand("[driveToSpeaker] No function!");
		/*
		Optional<Alliance> alliance = DriverStation.getAlliance();
		if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
			return driveToPose(Constants.Auto.ScoringPoses.BLU_SPEAKER.pose);
		} else if (alliance.isPresent() && alliance.get() == Alliance.Red) {
			return driveToPose(Constants.Auto.ScoringPoses.RED_SPEAKER.pose);
		}
		return new PrintCommand("[error] [driveToSpeaker] No Alliance Detected!");
		*/
	}

	/**
	 * Drives the robot to the amp depending on the alliance.
	 *
	 * @return The command to run to drive to the pose.
	 */
	public Command driveToAmp() {

		Optional<Alliance> alliance = DriverStation.getAlliance();
		
		if (alliance.get() == Alliance.Blue) {
			return driveToPose(Constants.Auto.ScoringPoses.BLU_AMP.pose);
		} else if (alliance.get() == Alliance.Red) {
			return driveToPose(Constants.Auto.ScoringPoses.RED_AMP.pose);
		}
		return new PrintCommand("[error] [driveToAmp] No Alliance Detected!");
	}

	/**
	 * Drives the robot on the path specified.
	 *
	 * @param name The name of the path to follow
	 * @param setOdomToStart Whether or not to reset the odometry to the start of the path
	 * @return The command to run to drive the path
	 */
	public Command drivePath(String name, boolean setOdomToStart) {
		PathPlannerPath path = PathPlannerPath.fromPathFile(name);
		if (setOdomToStart) {
			io.resetOdometry(new Pose2d(path.getPoint(0).position, io.getHeading()));
		}
		return AutoBuilder.followPath(path);
	}

	public void resetEmergencyOdometry() {
		io.resetOdometry(new Pose2d(0.0, 0.0, io.getYaw()));
	}

	public Pose2d getPose() {
		return io.getPose();
	}

	public Rotation2d getYaw() {
		return io.getYaw();
	}

	public void setSwerveCurrentLimit(int driveAmps, int turnAmps) {
		io.setDriveMotorCurrentLimit(driveAmps);
		io.setTurnMotorCurrentLimit(turnAmps);
	}

	public Pair<Rotation2d, Double> calculateAngleToSpeaker() {

		var alliance = DriverStation.getAlliance();

		Pose2d currentPose = io.getPose();
		Pose2d targetPose;

		double xDistanceMeters = currentPose.getX();
		double yDistanceMeters = currentPose.getY();
		double hDistanceMeters;

		double calculatedAngleRadians = 2.0 * Math.PI;

		// The correct ending angle for the TELEOP PID to check. Does not influence PathPlanner!
		double PIDOptimalEndingAngleDegrees = 0.0;

		// * --- RED ALLIANCE --- * //
		if (alliance.get() == Alliance.Blue) {
			targetPose = Constants.Auto.ScoringPoses.BLU_SPEAKER.pose;
			hDistanceMeters = PhotonUtils.getDistanceToPose(currentPose, targetPose);
			// If robot is ABOVE the amp with a middle tolerance of 0.5 meters
			if (yDistanceMeters > targetPose.getY()) {
				calculatedAngleRadians =
						Math.toRadians(90)
								- (Math.asin(xDistanceMeters / hDistanceMeters))
								+ Math.toRadians(180);
				PIDOptimalEndingAngleDegrees = currentPose.getRotation().getDegrees() + 360.0;

				// If robot is BELOW the amp with a middle tolerance of 0.5 meters
			} else if (yDistanceMeters < targetPose.getY()) {
				calculatedAngleRadians =
						(Math.toRadians(90) + (Math.asin(xDistanceMeters / hDistanceMeters)));
				PIDOptimalEndingAngleDegrees = currentPose.getRotation().getDegrees();
			}

			// * --- RED ALLIANCE --- * //
		} else if (alliance.get() == Alliance.Red) {
			targetPose = Constants.Auto.ScoringPoses.RED_SPEAKER.pose;
			hDistanceMeters = PhotonUtils.getDistanceToPose(currentPose, targetPose);

			/* Apply offset to the xDistanceMeters beacuse the measurements are taken off the opposite
			side of the field */
			xDistanceMeters = ScoringPoses.RED_SPEAKER.pose.getX() - currentPose.getX();

			// If robot is ABOVE the speaker with a middle tolerance of 0.5 meters
			if (yDistanceMeters > targetPose.getY() + 0.25) {
				calculatedAngleRadians =
						(Math.asin(xDistanceMeters / hDistanceMeters)) - Math.toRadians(90);

				// TODO: Test and correctly implement PIDOptimalEndingAngleDegrees for red alliance
				PIDOptimalEndingAngleDegrees = currentPose.getRotation().getDegrees();

				// If robot is BELOW the speaker with a middle tolerance of 0.5 meters
			} else if (yDistanceMeters < targetPose.getY() - 0.25) {
				calculatedAngleRadians =
						(Math.asin(xDistanceMeters / hDistanceMeters)) + Math.toRadians(135 + 180);

				PIDOptimalEndingAngleDegrees = currentPose.getRotation().getDegrees();
			}
			// * --- NO ALLIANCE! --- * //
		} else {
			DriverStation.reportError("[error] [calculateAngleToSpeaker] No Alliance Detected!", false);
			return new Pair<Rotation2d, Double>(new Rotation2d(), 0.0);
		}

		Logger.recordOutput("[calculateAngleToSpeaker] H Distance", hDistanceMeters);
		Logger.recordOutput("[calculateAngleToSpeaker] X Distance", xDistanceMeters);
		Logger.recordOutput(
				"[calculateAngleToSpeaker] Calculated Angle Radians", calculatedAngleRadians);
		Logger.recordOutput(
				"[calculateAngleToSpeaker] Calculated Angle Degrees",
				Math.toDegrees(calculatedAngleRadians));
		Logger.recordOutput(
				"[calculateAngleToSpeaker] Desired Pose",
				new Pose2d(currentPose.getTranslation(), new Rotation2d(calculatedAngleRadians)));
		Logger.recordOutput(
				"[calculateAngleToSpeaker] PID Optimal Ending Angle Degrees (Only for Teleop!)",
				PIDOptimalEndingAngleDegrees);

		return new Pair<Rotation2d, Double>(
				new Rotation2d(calculatedAngleRadians), PIDOptimalEndingAngleDegrees);
	}

	public Optional<Rotation2d> getRotationOverride() {
		if (Constants.ROTATION_OVERRIDE_STATE == RotationOverrideState.SPEAKER) {
			if (!DriverStation.isAutonomous()) {
				DriverStation.reportWarning(
						"[warning] Overriding for PathPlanner is only available in autonomous paths!", false);
			}
			return Optional.of(calculateAngleToSpeaker().getFirst());
		} else {
			// Return an empty for a disabled override
			return Optional.empty();
		}
	}

	public void driveRaw(Double translationX, Double translationY, Double angularRotationX) {
		io.drive(
				new Translation2d(
						translationX * io.getMaximumVelocity(), translationY * io.getMaximumVelocity()),
				angularRotationX * io.getMaximumAngularVelocity(),
				false,
				true);
	}

	public void driveJoystickHybrid(
			Double translationX, Double translationY, Double nonModifedAngular) {

		var alli = DriverStation.getAlliance();

		if (alli.get() == Alliance.Blue) {

			io.drive(

					// BLU
					new Translation2d(
							translationX * io.getMaximumVelocity(), -translationY * io.getMaximumVelocity()),
					nonModifedAngular * io.getMaximumAngularVelocity(),
					true,
					true);

		} else if (alli.get() == Alliance.Red) {

			io.drive(
					// RED
					new Translation2d(
							-translationX * io.getMaximumVelocity(), translationY * io.getMaximumVelocity()),
					nonModifedAngular * io.getMaximumAngularVelocity(),
					true,
					true);
		}
	}
}
