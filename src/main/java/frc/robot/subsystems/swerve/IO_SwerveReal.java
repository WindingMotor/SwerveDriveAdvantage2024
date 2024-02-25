// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import frc.robot.Constants.Vision;
import frc.robot.Constants.Vision.Camera;
import frc.robot.subsystems.vision.SUB_Vision;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveDriveConfiguration;
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
      swerveDrive = new SwerveParser(jsonDirectory).createSwerveDrive(maxSpeed);
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

  @Override
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  @Override
  public void resetOdometry(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }

  @Override
  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  @Override
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  @Override
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  @Override
  public PIDConstants getHeadingPID(){

    return new PIDConstants(
      swerveDrive.swerveController.config.headingPIDF.p, // Rotation PID
      swerveDrive.swerveController.config.headingPIDF.i,
      swerveDrive.swerveController.config.headingPIDF.d);


  }

  @Override
  public double getConfigurationRadius(){
    return swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters();
  }

  @Override
  public void drive(
      Translation2d translation, double rotation, boolean isFieldRelative, boolean isOpenLoop) {
    swerveDrive.drive(translation, rotation, isFieldRelative, isOpenLoop);
  }

  @Override
  public double getMaximumVelocity() {
    return swerveDrive.getMaximumVelocity();
  }

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
    //updateEstimationsForCamera(vision, Camera.LIMELIGHT);
  }

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
            if (!newTargets.isEmpty()){
              // Get the estimated pose and standard deviations and send them to the swerve drive
              // pose estimator
              var estPose = visionEst.get().estimatedPose.toPose2d();
              var estStdDevs = vision.getEstimationStdDevs(estPose, camera, newTargets);
              swerveDrive.addVisionMeasurement(
                  estPose, visionEst.get().timestampSeconds, estStdDevs);
              if(camera == Camera.LEFT_CAMERA){
                Logger.recordOutput("Left Vision Est", estPose);
              }else if(camera == Camera.RIGHT_CAMERA){
                Logger.recordOutput("Right Vision Est", estPose);
              }else{
                Logger.recordOutput("Limelight Vision Est", estPose);
              }
            }
          });
    }
  }

  @Override
  public void updateOdometry() {
    swerveDrive.updateOdometry();
  }
}
