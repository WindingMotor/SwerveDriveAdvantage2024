// Written by WindingMotor as part of the wmlib2j library.

package frc.robot.wmlib2j.vision;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.Constants.Vision.Camera;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;
public interface IO_VisionBase{


    @AutoLog
    public static class VisionInputs{

        public boolean leftCameraIsOn = false;
        public boolean rightCameraIsOn = false;
        public boolean intakeCameraIsOn = false;

        public boolean leftCameraHasTargets = false;
        public boolean rightCameraHasTargets = false;
        public boolean intakeCameraHasTargets = false;

        public double leftCameraLatency = 0.0;
        public double rightCameraLatency = 0.0;
        public double intakeCameraLatency = 0.0;
    }

    /**
     * Updates the inputs with the current values.
     * @param inputs The inputs to update.
    */
    void updateInputs(VisionInputs inputs);
    
    public PhotonPipelineResult getResult(Camera camera);
    
    public List<PhotonTrackedTarget> getTargets(Camera camera);

    public PhotonPoseEstimator getPoseEstimator(Camera camera);

}
