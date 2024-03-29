// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.Vision;
import frc.robot.Constants.Vision.Camera;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class IO_VisionReal implements IO_VisionBase {

	// Initialize cameras
	PhotonCamera leftCamera = new PhotonCamera("OV9281_02");
	PhotonCamera rightCamera = new PhotonCamera("OV9281_01");
	// PhotonCamera limelightCamera = new PhotonCamera("limelight");

	PhotonCamera lifecam = new PhotonCamera("Microsoft_LifeCam_HD-3000");

	double lastEstTimestamp = 0.0;

	// Initialize AprilTag field layout
	AprilTagFieldLayout fieldLayout;

	// Initialize pose estimators
	PhotonPoseEstimator leftPoseEstimator;
	PhotonPoseEstimator rightPoseEstimator;
	PhotonPoseEstimator limelightPoseEstimator;

	List<PhotonTrackedTarget> targets;

	public IO_VisionReal() {

		targets = new ArrayList<>();

		// Disable driver mode for the cameras
		leftCamera.setDriverMode(false);
		rightCamera.setDriverMode(false);
		//	limelightCamera.setDriverMode(false);

		// Try loading AprilTag field layout

		try {
			fieldLayout =
					AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
		} catch (IOException e) {
			DriverStation.reportError("Failed to load AprilTag field layout!", false);
		}

		// Initialize pose estimators
		leftPoseEstimator =
				new PhotonPoseEstimator(
						fieldLayout,
						PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
						leftCamera,
						Constants.Vision.Camera.LEFT_CAMERA.ROBOT_TO_CAMERA);
		rightPoseEstimator =
				new PhotonPoseEstimator(
						fieldLayout,
						PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
						rightCamera,
						Constants.Vision.Camera.RIGHT_CAMERA.ROBOT_TO_CAMERA);

		limelightPoseEstimator =
				new PhotonPoseEstimator(
						fieldLayout,
						PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
						rightCamera,
						Constants.Vision.Camera.LIMELIGHT.ROBOT_TO_CAMERA);

		// Set fallback strategies
		leftPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
		rightPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
		//		limelightPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
	}

	/**
	 * Updates the inputs with the current values.
	 *
	 * @param inputs The inputs to update.
	 */
	@Override
	public void updateInputs(VisionInputs inputs) {
		inputs.leftCameraIsOn = leftCamera.isConnected();
		inputs.rightCameraIsOn = rightCamera.isConnected();

		inputs.leftCameraHasTargets = leftCamera.getLatestResult().hasTargets();
		inputs.rightCameraHasTargets = rightCamera.getLatestResult().hasTargets();

		inputs.leftCameraLatency = leftCamera.getLatestResult().getLatencyMillis();
		inputs.rightCameraLatency = rightCamera.getLatestResult().getLatencyMillis();

		// inputs.limelightCameraIsOn = limelightCamera.isConnected();
		//	inputs.limelightCameraHasTargets = limelightCamera.getLatestResult().hasTargets();
	}

	/**
	 * Get the pose of the donut based on the latest result from the lifecam and trignometry.
	 *
	 * @return The pose of the donut
	 */
	public Pose2d getDonutPose() {
		if (lifecam.getLatestResult().hasTargets()) {

			// PhotonUtils.estimateCameraToTargetTranslation(targetDistMeters, targetYaw)

			double targetYaw = lifecam.getLatestResult().getBestTarget().getYaw();

			double targetPitch = lifecam.getLatestResult().getBestTarget().getPitch();

			double cameraPitchAngle = 7.0;
			double cameraHeightMeters = 0.3;

			double rDistanceMeters =
					1.0
							/ Math.tan(Math.toRadians(cameraPitchAngle) + Math.toRadians(targetPitch))
							* cameraHeightMeters;

			double xDistanceMeters = Math.cos(Math.toRadians(targetYaw)) * rDistanceMeters;
			double yDistanceMeters = Math.sin(Math.toRadians(targetYaw)) * rDistanceMeters;

			return new Pose2d(xDistanceMeters, yDistanceMeters, new Rotation2d());
		} else {
			return new Pose2d(-1, -1, new Rotation2d());
		}
	}

	/**
	 * Returns the latest result based on the given Camera.
	 *
	 * @param camera The camera to get the result from
	 * @return The latest result
	 */
	@Override
	public PhotonPipelineResult getResult(Camera camera) {
		if (camera == Constants.Vision.Camera.LEFT_CAMERA) {
			return leftCamera.getLatestResult();
		} else if (camera == Constants.Vision.Camera.RIGHT_CAMERA) {
			return rightCamera.getLatestResult();
		} else {
			// return limelightCamera.getLatestResult();
			return null;
		}
	}

	/**
	 * Retrieves the list of tracked targets from the specified camera.
	 *
	 * @param camera The camera object from which to retrieve the targets
	 * @return The list of tracked targets from the camera
	 */
	@Override
	public List<PhotonTrackedTarget> getTargets(Camera camera) {

		targets.clear();

		var latestResult = getResult(camera);

		if (latestResult.hasTargets()) {
			// Add camera targets to list
			for (PhotonTrackedTarget target : latestResult.getTargets()) {
				targets.add(target);
			}
			return targets;
		}
		return targets;
	}

	/**
	 * Returns the pose estimator for the specified camera.
	 *
	 * @param camera The camera to get the pose estimator for
	 * @return The pose estimator for the specified camera
	 */
	@Override
	public PhotonPoseEstimator getPoseEstimator(Camera camera) {
		if (camera == Constants.Vision.Camera.LEFT_CAMERA) {
			return leftPoseEstimator;
		} else if (camera == Constants.Vision.Camera.RIGHT_CAMERA) {
			return rightPoseEstimator;
		} else {
			return limelightPoseEstimator;
		}
	}

	/**
	 * Retrieves the estimated global pose from the specified camera.
	 *
	 * @param camera The camera from which to retrieve the estimated pose
	 * @return An optional containing the estimated robot pose, or empty if the camera is not valid
	 */
	public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Camera camera) {

		if (camera == Camera.LEFT_CAMERA) {
			var visionEst = leftPoseEstimator.update();
			double latestTimestamp = leftCamera.getLatestResult().getTimestampSeconds();
			boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;

			if (newResult) lastEstTimestamp = latestTimestamp;
			return visionEst;

		} else if (camera == Camera.RIGHT_CAMERA) {
			var visionEst = rightPoseEstimator.update();
			double latestTimestamp = rightCamera.getLatestResult().getTimestampSeconds();
			boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;

			if (newResult) lastEstTimestamp = latestTimestamp;
			return visionEst;
		} else { // Limelight
			return null;
		}
	}

	/**
	 * Calculate the estimation standard deviations based on the estimated pose.
	 *
	 * @param estimatedPose The estimated pose for calculation
	 * @param camera The camera to calculate the standard deviations for
	 * @return The calculated standard deviations
	 */
	public Matrix<N3, N1> getEstimationStdDevs(
			Pose2d estimatedPose, Camera camera, List<PhotonTrackedTarget> targets) {
		// Initialize standard deviations with the default values for a single tag
		var estStdDevs = Vision.SINGLE_TAG_STD_DEVS;

		int numTags = 0;
		double avgDist = 0;

		// Iterate over each target to calculate the average distance from the estimated pose
		for (var tgt : targets) {
			// Get the pose of the tag from the field tags
			var tagPose = rightPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
			if (camera == Constants.Vision.Camera.LEFT_CAMERA) {
				tagPose = leftPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
			}

			// if (camera == Constants.Vision.Camera.LIMELIGHT) {
			//	tagPose = limelightPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
			// }

			if (tagPose.isEmpty()) continue;
			numTags++;
			// Add the distance from the estimated pose to the tag's pose to the average distance
			avgDist +=
					tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
		}

		// If no tags were detected, return the default standard deviations
		if (numTags == 0) return estStdDevs;

		// Calculate the average distance
		avgDist /= numTags;

		// If multiple tags are detected, use the standard deviations for multiple tags
		if (numTags > 1) estStdDevs = Vision.MULTI_TAG_STD_DEVS;

		// If only one tag is detected and the average distance is greater than  4 meters, set high
		// standard deviations
		if (numTags == 1 && avgDist > 4) {
			estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
		} else {
			// Otherwise, scale the standard deviations based on the square of the average distance
			estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
		}

		// Return the calculated standard deviations
		return estStdDevs;
	}
}
