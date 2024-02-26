// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision.Camera;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

public class SUB_Vision extends SubsystemBase {

	private final IO_VisionBase io;

	public final VisionInputsAutoLogged inputs = new VisionInputsAutoLogged();

	public SUB_Vision(IO_VisionBase io) {
		this.io = io;
	}

	@Override
	public void periodic() {

		// Update the inputs.
		io.updateInputs(inputs);
	}

	/**
	 * Retrieves the estimated global pose of the robot using the specified camera and the previous
	 * estimated robot pose.
	 *
	 * @param camera The camera used for estimating the pose
	 * @return An Optional containing the estimated robot pose if the update is successful, otherwise
	 *     an empty Optional
	 */
	public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Camera camera) {
		return io.getEstimatedGlobalPose(camera);
	}

	/**
	 * Retrieves the estimation standard deviations for the given estimated pose.
	 *
	 * @param estimatedPose The estimated pose for which to retrieve standard deviations
	 * @return The estimation standard deviations for the given estimated pose
	 */
	public Matrix<N3, N1> getEstimationStdDevs(
			Pose2d estimatedPose, Camera camera, List<PhotonTrackedTarget> targets) {
		return io.getEstimationStdDevs(estimatedPose, camera, targets);
	}

	/**
	 * Retrieves the ambiguity value for the given target ID from the cameras.
	 *
	 * @param id the ID of the target
	 * @return the ambiguity value, or 1.0 if the target ID is not found
	 */
	public Double getAmbiguity(int id) {

		for (PhotonTrackedTarget tar : io.getTargets(Camera.LEFT_CAMERA)) {

			if (tar.getFiducialId() == id) {
				return tar.getPoseAmbiguity();
			}
		}

		for (PhotonTrackedTarget tar : io.getTargets(Camera.RIGHT_CAMERA)) {

			if (tar.getFiducialId() == id) {
				return tar.getPoseAmbiguity();
			}
		}
		return 1.0;
	}
}
