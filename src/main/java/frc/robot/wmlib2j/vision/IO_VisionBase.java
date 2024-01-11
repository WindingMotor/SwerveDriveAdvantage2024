
package frc.robot.wmlib2j.vision;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Constants.Vision.Camera;

import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.LogTable;


public interface IO_VisionBase{

    class VisionInputs implements LoggableInputs{

        public boolean leftCameraIsOn = false;
        public boolean rightCameraIsOn = false;
        public boolean intakeCameraIsOn = false;

        public boolean leftCameraHasTargets = false;
        public boolean rightCameraHasTargets = false;
        public boolean intakeCameraHasTargets = false;

        public List<PhotonTrackedTarget> leftCameraTargets = new ArrayList<>();
        public List<PhotonTrackedTarget> rightCameraTargets = new ArrayList<>();
        public List<PhotonTrackedTarget> intakeCameraTargets = new ArrayList<>();

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

            table.put("leftCameraTargets", leftCameraTargets);
            table.put("rightCameraTargets", rightCameraTargets);
            table.put("intakeCameraTargets", intakeCameraTargets);

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

            leftCameraTargets = table.get("leftCameraTargets", leftCameraTargets);
            rightCameraTargets = table.get("rightCameraTargets", rightCameraTargets);
            intakeCameraTargets = table.get("intakeCameraTargets", intakeCameraTargets);

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
