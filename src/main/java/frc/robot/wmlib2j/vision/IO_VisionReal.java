
package frc.robot.wmlib2j.vision;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import frc.robot.Constants.Vision.Camera;


public class IO_VisionReal implements IO_VisionBase{

    PhotonCamera leftCamera = new PhotonCamera("OV9281_02");
    PhotonCamera rightCamera = new PhotonCamera("OV9281_01");

    AprilTagFieldLayout fieldLayout;

    PhotonPoseEstimator leftPoseEstimator;
    PhotonPoseEstimator rightPoseEstimator;

    public IO_VisionReal(){

        leftCamera.setDriverMode(false);
        rightCamera.setDriverMode(false);

        try{
            fieldLayout = AprilTagFieldLayout.loadFromResource(Filesystem.getDeployDirectory() + "/2024-crescendo.json");
        }catch(IOException e){
            DriverStation.reportError("Failed to load AprilTag field layout!", false);
        }

        leftPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, leftCamera, Constants.Vision.Camera.LEFT_CAMERA.ROBOT_TO_CAMERA);
        rightPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, rightCamera, Constants.Vision.Camera.RIGHT_CAMERA.ROBOT_TO_CAMERA);

        leftPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
        rightPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);

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
        inputs.rightCameraLatency = leftCamera.getLatestResult().getLatencyMillis();
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


}
