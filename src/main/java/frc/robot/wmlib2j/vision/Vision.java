
package frc.robot.wmlib2j.vision;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {

    private final IO_VisionBase io;

    private List<PhotonTrackedApriltag> targets = new ArrayList<>();

    AprilTagFieldLayout fieldLayout;
    
    public Vision(IO_VisionBase io){
        this.io = io;
        
        try{
            fieldLayout = AprilTagFieldLayout.loadFromResource(Filesystem.getDeployDirectory() + "/2024-crescendo.json");
        }catch(IOException e){
            DriverStation.reportError("Failed to load AprilTag field layout!", false);
        }

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

        // Print out the seen targets, for testing
        List<String> ids = new ArrayList<>();
        for(PhotonTrackedApriltag target : targets){
            ids.add(target.getId() + target.camera.CAMERA_NAME + target.getTranslationMeters().toString());
        }
        SmartDashboard.putString("Seen IDs", ids.toString());
        SmartDashboard.putNumber("X Dist ID2", getRobotCenterDistanceToTag(2).getX());

    }

    /**
     * Retrieves the robot's center distance to a specific tag with camera offsets applied.
     * @param  id  The fiducial ID of the tag
     * @return     The Pose3d of the robot's distance to the tag.
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