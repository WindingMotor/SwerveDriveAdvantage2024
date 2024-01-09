
package frc.robot.wmlib2j.vision;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


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
    public void updateInputs(VisionInputs inputs){
        inputs.leftCameraIsOn = leftCamera.isConnected();
        inputs.rightCameraIsOn = rightCamera.isConnected();

        inputs.leftCameraHasTargets = leftCamera.getLatestResult().hasTargets();
        inputs.rightCameraHasTargets = rightCamera.getLatestResult().hasTargets();

        inputs.leftCameraTargets = getLeftTargets();
        inputs.rightCameraTargets = getRightTargets();

        inputs.leftCameraLatency = leftCamera.getLatestResult().getLatencyMillis();
        inputs.rightCameraLatency = leftCamera.getLatestResult().getLatencyMillis();
    }

    public PhotonPipelineResult getLeftCameraResult(){
        return leftCamera.getLatestResult();
    }

    public PhotonPipelineResult getRightCameraResult(){
        return rightCamera.getLatestResult();
    }

    public List<PhotonTrackedTarget> getLeftTargets(){
        List<PhotonTrackedTarget> leftTargets = new ArrayList<>();

        // Add camera targets to list
        for(PhotonTrackedTarget target : leftCamera.getLatestResult().getTargets()){
            leftTargets.add(target);
        }

        return leftTargets;
    }

    public List<PhotonTrackedTarget> getRightTargets(){
        List<PhotonTrackedTarget> rightTargets = new ArrayList<>();

        // Add camera targets to list
        for(PhotonTrackedTarget target : rightCamera.getLatestResult().getTargets()){
            rightTargets.add(target);
        }

        return rightTargets;
    }



}
