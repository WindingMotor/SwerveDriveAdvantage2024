// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode;
import frc.robot.subsystems.vision.SUB_Vision;
import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class SUB_Swerve extends SubsystemBase {

	private final IO_SwerveBase io;

	private final SwerveInputsAutoLogged inputs = new SwerveInputsAutoLogged();

	private final SUB_Vision vision;

	// Odometry lock, prevents updates while reading data
	private static final Lock odometryLock = new ReentrantLock();

	private PIDController snapAnglePID;

	public SUB_Swerve(IO_SwerveBase io, SUB_Vision vision) {
		this.io = io;
		this.vision = vision;

		AutoBuilder.configureHolonomic(
				io::getPose, // Gets current robot pose
				io::resetOdometry, // Resets robot odometry if path has starter pose
				io::getRobotVelocity, // Gets chassis speed in robot relative
				io::setChassisSpeeds, // Drives the robot in robot relative chassis speeds
				new HolonomicPathFollowerConfig(
						new PIDConstants(5.0, 0.0, 0.0, 0.0), // Translation
						new PIDConstants(7.0, 0.0, 0.0, 0.0), // Heading
						/*
						 * IMPORTANT NOTE: These auto PIDs only have a relatively small effect. For a larger and better effect use the YAGSL PIDF config.
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

		snapAnglePID = new PIDController(1.0, 0.0, 0.0);
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

	public void driveRaw(Double translationX, Double translationY, Double angularRotationX) {
		io.drive(
				new Translation2d(
						translationX * io.getMaximumVelocity(), translationY * io.getMaximumVelocity()),
				angularRotationX * io.getMaximumAngularVelocity(),
				false,
				true);
	}

	/**
	 * Drives the robot, in field-relative, based of the specified inputs.
	 *
	 * @param translationX A supplier for the X translation
	 * @param translationY A supplier for the Y translation
	 * @param angularRotationX A supplier for the angular rotation
	 * @return The command for driving the swerve
	 */
	public Command driveJoystick(
			DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {

		return run(
				() -> {
					double newAngularRotation = angularRotationX.getAsDouble();
					if (false) {
						// Find the closest snap point based off current robot degrees. It should be able to
						// snap to 0deg, 90deg, 180deg, and 270deg.
						double currentYaw = io.getYaw().getDegrees();
						double closestSnapAngle = 0;
						double minDifference = Math.abs(currentYaw - closestSnapAngle);

						double[] snapAngles = {0, 90, 180, 270};
						for (double angle : snapAngles) {
							double difference = Math.abs(currentYaw - angle);
							if (difference < minDifference) {
								minDifference = difference;
								closestSnapAngle = angle;
							}
						}
						newAngularRotation = snapAnglePID.calculate(io.getYaw().getDegrees(), closestSnapAngle);
					}

					var alli = DriverStation.getAlliance();

					if (alli.get() == Alliance.Blue) {

						io.drive(

								// BLU
								new Translation2d(
										translationX.getAsDouble() * io.getMaximumVelocity(),
										-translationY.getAsDouble() * io.getMaximumVelocity()),
								-newAngularRotation * io.getMaximumAngularVelocity(),
								true,
								true);

					} else if (alli.get() == Alliance.Red) {

						io.drive(
								// RED
								new Translation2d(
										-translationX.getAsDouble() * io.getMaximumVelocity(),
										translationY.getAsDouble() * io.getMaximumVelocity()),
								-newAngularRotation * io.getMaximumAngularVelocity(),
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
								1,
								6,
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
		Optional<Alliance> alliance = DriverStation.getAlliance();
		if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
			return driveToPose(Constants.Auto.ScoringPoses.BLU_SPEAKER.pose);
		} else if (alliance.isPresent() && alliance.get() == Alliance.Red) {
			return driveToPose(Constants.Auto.ScoringPoses.RED_SPEAKER.pose);
		}
		return new PrintCommand("[error] [driveToSpeaker] No Alliance Detected!");
	}

	/**
	 * Drives the robot to the amp depending on the alliance.
	 *
	 * @return The command to run to drive to the pose.
	 */
	public Command driveToAmp() {
		return driveToPose(Constants.Auto.ScoringPoses.BLU_AMP.pose);
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
}
