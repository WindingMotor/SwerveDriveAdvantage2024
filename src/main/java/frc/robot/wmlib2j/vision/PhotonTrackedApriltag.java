
package frc.robot.wmlib2j.vision;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.Vision.Camera;

public class PhotonTrackedApriltag{

    double id;
    Camera camera;
    
    Translation3d translation3d;
    Rotation3d rotation3d;

    public PhotonTrackedApriltag(PhotonTrackedTarget apriltag, Camera camera){

        id = apriltag.getFiducialId();
        this.camera = camera;

        translation3d = apriltag.getBestCameraToTarget().getTranslation();
        rotation3d = apriltag.getBestCameraToTarget().getRotation();
        
    }

    public double getId(){ return id; }
    public Camera getCamera(){ return camera; }

    public Translation3d getTranslationMeters(){ return translation3d; }
    public Rotation3d getRotationRadians(){ return rotation3d; }

}