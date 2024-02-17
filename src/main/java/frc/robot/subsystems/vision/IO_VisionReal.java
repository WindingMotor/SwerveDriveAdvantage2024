// Written by WindingMotor as part of the wmlib2j library.

package frc.robot.subsystems.vision;
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
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.Vision.Camera;


public class IO_VisionReal implements IO_VisionBase{

    // Initialize cameras
    PhotonCamera leftCamera = new PhotonCamera("OV9281_02");
    PhotonCamera rightCamera = new PhotonCamera("OV9281_01");

    double lastEstTimestamp = 0.0;

    // Initialize AprilTag field layout
    AprilTagFieldLayout fieldLayout;

    // Initialize pose estimators
    PhotonPoseEstimator leftPoseEstimator;
    PhotonPoseEstimator rightPoseEstimator;

    public IO_VisionReal(){

        // Disable driver mode for the cameras
        leftCamera.setDriverMode(false);
        rightCamera.setDriverMode(false);

        // Try loading AprilTag field layout
        
        try{
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        }catch(IOException e){
            DriverStation.reportError("Failed to load AprilTag field layout!", false);
        }
        

        // Initialize pose estimators
        leftPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, leftCamera, Constants.Vision.Camera.LEFT_CAMERA.ROBOT_TO_CAMERA);
        rightPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, rightCamera, Constants.Vision.Camera.RIGHT_CAMERA.ROBOT_TO_CAMERA);

        // Set fallback strategies
        leftPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        rightPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);


    }

    /**
     * Updates the inputs with the current values.
     * @param inputs The inputs to update.
    */
    @Override
    public void updateInputs(VisionInputs inputs){
        inputs.leftCameraIsOn = leftCamera.isConnected();
        inputs.rightCameraIsOn = rightCamera.isConnected();

        inputs.leftCameraHasTargets = leftCamera.getLatestResult().hasTargets();
        inputs.rightCameraHasTargets = rightCamera.getLatestResult().hasTargets();

        inputs.leftCameraLatency = leftCamera.getLatestResult().getLatencyMillis();
        inputs.rightCameraLatency = rightCamera.getLatestResult().getLatencyMillis();
    }

    /**
     * Returns the latest result based on the given Camera.
     * @param  camera  The camera to get the result from
     * @return         The latest result
    */
    @Override
    public PhotonPipelineResult getResult(Camera camera){
        if(camera == Constants.Vision.Camera.LEFT_CAMERA){
            return leftCamera.getLatestResult();
        }else{
            return rightCamera.getLatestResult();
        }
    }

    /**
     * Retrieves the list of tracked targets from the specified camera.
     * @param  camera  The camera object from which to retrieve the targets
     * @return         The list of tracked targets from the camera
    */
    @Override
    public List<PhotonTrackedTarget> getTargets(Camera camera){

        List<PhotonTrackedTarget> targets = new ArrayList<>();
        var latestResult = getResult(camera);

        if(latestResult.hasTargets()){
            // Add camera targets to list
            for(PhotonTrackedTarget target : latestResult.getTargets()){
                targets.add(target);
            }
            return targets;
        }
        return new ArrayList<>();
    }

    /**
     * Returns the pose estimator for the specified camera.
     * @param  camera  The camera to get the pose estimator for
     * @return         The pose estimator for the specified camera
    */
    @Override
    public PhotonPoseEstimator getPoseEstimator(Camera camera){
        if(camera == Constants.Vision.Camera.LEFT_CAMERA){
            return leftPoseEstimator;
        }else{
            return rightPoseEstimator;
        }
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Camera camera) {

        if(camera == Camera.LEFT_CAMERA){
            var visionEst = leftPoseEstimator.update();
            if(!visionEst.isEmpty()){
                 double latestTimestamp = leftCamera.getLatestResult().getTimestampSeconds();
                boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) < 1e-2;

                 if (newResult) lastEstTimestamp = latestTimestamp;
                 return visionEst;
        }
    }

        else{
            var visionEst = rightPoseEstimator.update();
            if(!visionEst.isEmpty()){
            double latestTimestamp = rightCamera.getLatestResult().getTimestampSeconds();
            boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;

            if (newResult) lastEstTimestamp = latestTimestamp;
            return visionEst;
            }
        }
        return null;

    }




        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);



    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = kSingleTagStdDevs;
        var targets = leftCamera.getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = leftPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

}
