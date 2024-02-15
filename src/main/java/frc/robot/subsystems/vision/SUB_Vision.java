// Written by WindingMotor as part of the wmlib2j library.

package frc.robot.subsystems.vision;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Vision.Camera;

public class SUB_Vision extends SubsystemBase {

    private  final IO_VisionBase io;

    public final VisionInputsAutoLogged inputs = new VisionInputsAutoLogged();

    private List<PhotonTrackedApriltag> targets = new ArrayList<>();

    public SUB_Vision(IO_VisionBase io){
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

        // Update the inputs.
        io.updateInputs(inputs);

        // Process inputs and send to logger.
        Logger.processInputs("Vision", inputs);
        Logger.recordOutput("ID 7", getDirectDistanceMetersToTag(7));
        Logger.recordOutput("Targets", targets.toString());
    }

    /**
     * Retrieves the PhotonTrackedApriltag with the specified id.
     * @param  id   The id of the PhotonTrackedApriltag to retrieve
     * @return      The PhotonTrackedApriltag with the specified id, or null if not found
     */
    public PhotonTrackedApriltag getTag(int id){
        for(PhotonTrackedApriltag target : targets){
            if(target.getId() == id){
                return target;
            }
        }
        return null;
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
     * Get the distance in meters to a specific tag.
     * @param  id   The ID of the tag
     * @return     The direct distance in meters to the tag
     */
    public Double getDirectDistanceMetersToTag(int id){
        PhotonTrackedApriltag target = getTag(id);
        double distance = 0.0;

        if(target != null){
            // Get the height of the camera and the height of the target
            double cameraHeight = target.getCamera().ROBOT_TO_CAMERA.getTranslation().getZ();
            double targetHeight = target.getTranslationMeters().getZ();

            // Get the pitch of the camera and the pitch of the target
            double cameraPitch = target.getCamera().ROBOT_TO_CAMERA.getRotation().getAngle();
            double targetPitch = target.getRotationRadians().getAngle();

            // Get the distance from the camera to the target
            distance = (targetHeight - cameraHeight) / Math.tan(cameraPitch - targetPitch);
        }
        return distance;
    }

    /**
     * Retrieves the estimated global pose of the robot using the specified camera and the previous estimated robot pose.
     * @param  camera                      The camera used for estimating the pose
     * @return                             An Optional containing the estimated robot pose if the update is successful, otherwise an empty Optional
    */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Camera camera){
        //io.getPoseEstimator(camera).setReferencePose(previousEstimatedRobotPose);
        return io.getPoseEstimator(camera).update();
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