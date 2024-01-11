
package frc.robot.wmlib2j.vision;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Constants;
import frc.robot.Constants.Vision.Camera;


public class IO_VisionReal implements IO_VisionBase{

    PhotonCamera leftCamera = new PhotonCamera("OV9281_02");
    PhotonCamera rightCamera = new PhotonCamera("OV9281_01");

    public IO_VisionReal(){

        leftCamera.setDriverMode(false);
        rightCamera.setDriverMode(false);

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

        inputs.leftCameraTargets = getResult(Camera.LEFT_CAMERA).targets;
        inputs.rightCameraTargets = getResult(Camera.RIGHT_CAMERA).targets;

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
