
package frc.robot.wmlib2j.vision;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {

    private final IO_VisionBase io;
    private final IO_VisionBase.VisionInputs inputs = new IO_VisionBase.VisionInputs();

    private List<PhotonTrackedApriltag> targets = new ArrayList<>();
    
    private Optional<EstimatedRobotPose> lastEstimatedRobotPose;

    public Vision(IO_VisionBase io){
        this.io = io;

    }

    @Override
    public void periodic(){

        targets.clear();

        // Get latest left and right targets
        List<PhotonTrackedTarget> leftTargets = io.getTargets(Constants.Vision.Camera.LEFT_CAMERA);
        List<PhotonTrackedTarget> rightTargets = io.getTargets(Constants.Vision.Camera.RIGHT_CAMERA);

        // Combine targets from cameras to one list
        for(PhotonTrackedTarget target : leftTargets){
            targets.add(
                new PhotonTrackedApriltag(target, Constants.Vision.Camera.LEFT_CAMERA)
            );
        }

        for(PhotonTrackedTarget target : rightTargets){
            targets.add(
                new PhotonTrackedApriltag(target, Constants.Vision.Camera.RIGHT_CAMERA)
            );
        }

        // Remove duplicates targets with same fiducial ID
        for(int i = 0; i < targets.size(); i++){
            PhotonTrackedApriltag target = targets.get(i);
            for(int j = i + 1; j < targets.size(); j++){
                if(targets.get(j).getId() == target.getId()){
                    targets.remove(j);
                    j--; // adjust index after removal
                }
            }
        }

        /* 
        // Print out the seen targets, for testing
        List<String> ids = new ArrayList<>();
        for(PhotonTrackedApriltag target : targets){
            ids.add(target.getId() + target.camera.CAMERA_NAME + target.getTranslationMeters().toString());
        }
        SmartDashboard.putString("Seen IDs", ids.toString());
        SmartDashboard.putNumber("X Dist ID2", getRobotCenterDistanceToTag(2).getX());
        */

        // Update the inputs.
        io.updateInputs(inputs);

        // Process inputs and send to logger.
        Logger.processInputs("Vision", inputs);


    }

    /**
     * Retrieves the robot's center distance to a specific tag with camera offsets applied.
     * @param  id  The fiducial ID of the tag
     * @return     The Pose3d of the robot's distance to the tag in meters and radians.
     */
    public Pose3d getRobotCenterDistanceToTag(int id){
        for(PhotonTrackedApriltag target : targets){
            if(target.getId() == id){
                    return new Pose3d(
                        target.getTranslationMeters().minus(target.camera.ROBOT_TO_CAMERA.getTranslation()),
                        target.getRotationRadians()
                    );
            }
        }
        return new Pose3d();
    }


    /**
     * Converts inches to meters.
     * @param  inches	The length in inches to be converted
     * @return         	The length in meters
     */
    public static double inchesToMeters(double inches){
        return inches * 0.0254;
    }
    

}