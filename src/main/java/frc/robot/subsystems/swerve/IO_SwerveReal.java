// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import frc.robot.Constants.Vision;
import frc.robot.Constants.Vision.Camera;
import frc.robot.subsystems.vision.SUB_Vision;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/** Represents a real implementation of the shooter. */
public class IO_SwerveReal implements IO_SwerveBase {

	private static double maxSpeed = Units.feetToMeters(4.5);
	private static File jsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
	private static SwerveDrive swerveDrive;

	private static List<PhotonTrackedTarget> newTargets;
	private static List<Pose3d> targetPoses;

	public IO_SwerveReal() {

		// Initialize the swerve drive based of the json file
		try {
			swerveDrive =
					new SwerveParser(jsonDirectory).createSwerveDrive(Constants.Maestro.MAX_MODULE_SPEED_MPS);
		} catch (IOException e) {
			e.printStackTrace();
		}

		newTargets = new ArrayList<>();
		targetPoses = new ArrayList<>();

		// Set the telemetry verbosity to high for debugging
		SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

		// Should only be enabled when controlling the robot via angle
		swerveDrive.setHeadingCorrection(false);
	}

	/**
	 * Updates the inputs with the current values.
	 *
	 * @param inputs The inputs to update.
	 */
	@Override
	public void updateInputs(SwerveInputs inputs) {
		inputs.pose = swerveDrive.getPose();
		inputs.yaw = swerveDrive.getYaw();
		inputs.odometryHeading = swerveDrive.getOdometryHeading();
	}

	/**
	 * Retrieves the current pose of the swerve drive.
	 *
	 * @return The pose of the swerve drive
	 */
	@Override
	public Pose2d getPose() {
		return swerveDrive.getPose();
	}

	/**
	 * Resets the odometry to the given pose.
	 *
	 * @param pose The pose to reset the odometry to
	 */
	@Override
	public void resetOdometry(Pose2d pose) {
		swerveDrive.resetOdometry(pose);
	}

	/**
	 * Gets the robot's velocity.
	 *
	 * @return The current swerve drive velocity in chassis speeds
	 */
	@Override
	public ChassisSpeeds getRobotVelocity() {
		return swerveDrive.getRobotVelocity();
	}

	/**
	 * Sets the chassis speeds for the swerve drive.
	 *
	 * @param chassisSpeeds The desired chassis speeds in MPS
	 */
	@Override
	public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
		swerveDrive.setChassisSpeeds(chassisSpeeds);
	}

	/**
	 * Retrieves the current headin of the robot from its pose.
	 *
	 * @return The current heading in a Rotation2d
	 */
	@Override
	public Rotation2d getHeading() {
		return getPose().getRotation();
	}

	/**
	 * Retrieves the current swerve drive heading PID constants.
	 *
	 * @return The heading PID constants
	 */
	@Override
	public PIDConstants getHeadingPID() {

		return new PIDConstants(
				swerveDrive.swerveController.config.headingPIDF.p, // Rotation PID
				swerveDrive.swerveController.config.headingPIDF.i,
				swerveDrive.swerveController.config.headingPIDF.d);
	}

	/**
	 * Retrieves the configuration radius of the swerve drive.
	 *
	 * @return the configuration radius in meters
	 */
	@Override
	public double getConfigurationRadius() {
		return swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters();
	}

	/**
	 * Drive the robot using the given translation, rotation, and mode.
	 *
	 * @param translation The desired translation of the robot
	 * @param rotation The desired rotation of the robot
	 * @param isFieldRelative Whether the driving mode is field relative
	 * @param isOpenLoop Whether the driving mode is open loop
	 */
	@Override
	public void drive(
			Translation2d translation, double rotation, boolean isFieldRelative, boolean isOpenLoop) {
		swerveDrive.drive(translation, rotation, isFieldRelative, isOpenLoop);
	}

	/**
	 * Get the maximum swerve velocity.
	 *
	 * @return The maximum velocity
	 */
	@Override
	public double getMaximumVelocity() {
		return swerveDrive.getMaximumVelocity();
	}

	/**
	 * Retrieves the maximum swerve angular velocity.
	 *
	 * @return The maximum angular velocity
	 */
	@Override
	public double getMaximumAngularVelocity() {
		return swerveDrive.getMaximumAngularVelocity();
	}

	/**
	 * Updates the swerve drive's estimations based on vision measurements from the specified camera.
	 * Only adds the measurement if it's not ambiguous.
	 *
	 * @param vision The vision subsystem providing the measurements.
	 */
	@Override
	public void updateEstimations(SUB_Vision vision) {
		// Update estimations for cameras.
		updateEstimationsForCamera(vision, Camera.LEFT_CAMERA);
		updateEstimationsForCamera(vision, Camera.RIGHT_CAMERA);
		// updateEstimationsForCamera(vision, Camera.LIMELIGHT);
	}

	/**
	 * Update estimations for the camera using photon vision data.
	 *
	 * @param vision The SUB_Vision instance
	 * @param camera The Camera instance
	 */
	private void updateEstimationsForCamera(SUB_Vision vision, Camera camera) {

		// Get estimated pose of camera from photon vision
		var visionEst = vision.getEstimatedGlobalPose(camera);

		// Create an empty array of targets poses for logging
		targetPoses.clear();

		// Check if pose is not null
		if (visionEst != null) {

			// Make a new empy array of targets
			newTargets.clear();

			// If estimate is present
			visionEst.ifPresent(
					est -> {

						// Loop through targets and only add targets that are not ambiguous
						for (PhotonTrackedTarget tar : est.targetsUsed) {
							if (tar.getPoseAmbiguity() < Vision.MIN_AMBUGUITY) {
								// Add target to newTargets array
								newTargets.add(tar);
								// Add target pose to targetPoses array
								targetPoses.add(
										new Pose3d(
												tar.getBestCameraToTarget().getTranslation(),
												tar.getBestCameraToTarget().getRotation()));
							}
						}

						// If the new targets array is not empty
						if (!newTargets.isEmpty()) {
							// Get the estimated pose and standard deviations and send them to the swerve drive
							// pose estimator
							var estPose = visionEst.get().estimatedPose.toPose2d();
							var estStdDevs = vision.getEstimationStdDevs(estPose, camera, newTargets);
							swerveDrive.addVisionMeasurement(
									estPose, visionEst.get().timestampSeconds, estStdDevs);
							if (camera == Camera.LEFT_CAMERA) {
								Logger.recordOutput("Left Vision Est", estPose);
							} else if (camera == Camera.RIGHT_CAMERA) {
								Logger.recordOutput("Right Vision Est", estPose);
							} else {
								Logger.recordOutput("Limelight Vision Est", estPose);
							}
						}
					});
		}
	}

	/** Updates the odometry for the swerve drive. */
	@Override
	public void updateOdometry() {
		swerveDrive.updateOdometry();
	}

	/**
	 * Sets the current limit for each drive drive motor in the swerve.
	 *
	 * @param currentLimit The new current limit to be set
	 */
	@Override
	public void setDriveMotorCurrentLimit(int currentLimit) {
		for (SwerveModule module : swerveDrive.getModules()) {
			module.getDriveMotor().setCurrentLimit(currentLimit);
		}
	}
	/**
	 * Sets the brake mode for all swerve modules in the swerve drive configuration.
	 *
	 * @param isBrakeMode True to enable brake mode, false to disable
	 */
	@Override
	public void setBrakeMode(boolean isBrakeMode) {
		for (SwerveModule module : swerveDrive.swerveDriveConfiguration.modules) {
			module.setMotorBrake(isBrakeMode);
		}
	}

	/** Sets the angle of all swerve modules to 0.0. */
	@Override
	public void setZero() {
		for (SwerveModule module : swerveDrive.swerveDriveConfiguration.modules) {
			module.setAngle(0.0);
		}
	}

	/** Sets the lock for the swerve drive by setting the motor brake and angle */
	@Override
	public void setLock() {
		for (SwerveModule module : swerveDrive.swerveDriveConfiguration.modules) {
			module.setMotorBrake(true);
			module.setAngle(45);
		}
	}
}
