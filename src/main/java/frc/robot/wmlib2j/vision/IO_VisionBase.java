
package frc.robot.wmlib2j.vision;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.Vision.Camera;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.LogTable;


public interface IO_VisionBase{

    class VisionInputs implements LoggableInputs{

        public boolean leftCameraIsOn = false;
        public boolean rightCameraIsOn = false;
        public boolean intakeCameraIsOn = false;

        public boolean leftCameraHasTargets = false;
        public boolean rightCameraHasTargets = false;
        public boolean intakeCameraHasTargets = false;

        public double leftCameraLatency = 0.0;
        public double rightCameraLatency = 0.0;
        public double intakeCameraLatency = 0.0;

        public void toLog(LogTable table){
            table.put("leftCameraIsOn", leftCameraIsOn);
            table.put("rightCameraIsOn", rightCameraIsOn);
            table.put("intakeCameraIsOn", intakeCameraIsOn);

            table.put("leftCameraHasTargets", leftCameraHasTargets);
            table.put("rightCameraHasTargets", rightCameraHasTargets);
            table.put("intakeCameraHasTargets", intakeCameraHasTargets);

            table.put("leftCameraLatency", leftCameraLatency);
            table.put("rightCameraLatency", rightCameraLatency);
            table.put("intakeCameraLatency", intakeCameraLatency);
        }

        public void fromLog(LogTable table){
            leftCameraIsOn = table.get("leftCameraIsOn", leftCameraIsOn);
            rightCameraIsOn = table.get("rightCameraIsOn", rightCameraIsOn);
            intakeCameraIsOn = table.get("intakeCameraIsOn", intakeCameraIsOn);

            leftCameraHasTargets = table.get("leftCameraHasTargets", leftCameraHasTargets);
            rightCameraHasTargets = table.get("rightCameraHasTargets", rightCameraHasTargets);
            intakeCameraHasTargets = table.get("intakeCameraHasTargets", intakeCameraHasTargets);

            leftCameraLatency = table.get("leftCameraLatency", leftCameraLatency);
            rightCameraLatency = table.get("rightCameraLatency", rightCameraLatency);
            intakeCameraLatency = table.get("intakeCameraLatency", intakeCameraLatency);
        }
    }

    /**
     * Updates the inputs with the current values.
     * @param inputs The inputs to update.
    */
    void updateInputs(VisionInputs inputs);
    
    public PhotonPipelineResult getResult(Camera camera);
    
    public List<PhotonTrackedTarget> getTargets(Camera camera);


}
