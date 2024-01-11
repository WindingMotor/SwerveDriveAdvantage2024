
package frc.robot.wmlib2j.vision;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.Vision.Camera;

/**
 * Represents a tracked AprilTag detected by the PhotonVision library.
 * Each instance of this class corresponds to a unique AprilTag detected by the robot's cameras.
*/
public class PhotonTrackedApriltag{

    double id;
    Camera camera;
    
    Translation3d translation3d;
    Rotation3d rotation3d;

    /**
    * Constructs a new PhotonTrackedApriltag object.
    * @param apriltag The PhotonTrackedTarget object representing the detected AprilTag.
    * @param camera The camera that detected the AprilTag.
    */
    public PhotonTrackedApriltag(PhotonTrackedTarget apriltag, Camera camera){

        id = apriltag.getFiducialId();
        this.camera = camera;

        translation3d = apriltag.getBestCameraToTarget().getTranslation();
        rotation3d = apriltag.getBestCameraToTarget().getRotation();

    }

    /**
    * Returns the fiducial ID of the AprilTag.
    * @return The fiducial ID of the AprilTag.
    */
    public double getId(){ return id; }

    /**
    * Returns the camera that detected the AprilTag.
    * @return The camera that detected the AprilTag.
    */
    public Camera getCamera(){ return camera; }

    /**
    * Returns the 3D translation of the AprilTag relative to the camera.
    * @return The 3D translation of the AprilTag relative to the camera.
    */
    public Translation3d getTranslationMeters(){ return translation3d; }

    /**
    * Returns the 3D rotation of the AprilTag relative to the camera.
    * @return The 3D rotation of the AprilTag relative to the camera.
    */
    public Rotation3d getRotationRadians(){ return rotation3d; }

}