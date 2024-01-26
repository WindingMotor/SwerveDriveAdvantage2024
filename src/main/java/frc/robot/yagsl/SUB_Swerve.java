
package frc.robot.yagsl;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Swerve extends SubsystemBase{

    private final IO_SwerveBase io;

    private final SwerveInputsAutoLogged inputs = new SwerveInputsAutoLogged();

    //private final Vision vision;

    public SUB_Swerve(IO_SwerveBase io){
        this.io = io;
        //this.vision = vision;

        // Configure PathPlanner
        io.setupPathPlanner(this);
    }

    public void periodic(){

        // Add vision measurements
        //io.addVisionMeasurement(vision.getEstimatedGlobalPose(Constants.Vision.Camera.LEFT_CAMERA).get());
        //io.addVisionMeasurement(vision.getEstimatedGlobalPose(Constants.Vision.Camera.RIGHT_CAMERA).get());

        // Update the odometry
        io.updateOdometry();

        // Update the inputs.
        io.updateInputs(inputs);

        // Process inputs and send to logger.
        Logger.processInputs("Swerve", inputs);
    }

    /**
     * Drives the robot, in field-relative, based of the specified inputs.
     * @param  translationX      A supplier for the X translation
     * @param  translationY      A supplier for the Y translation
     * @param  angularRotationX  A supplier for the angular rotation
     * @return                   The command for driving the swerve drive
    */
    public Command driveSwerve(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX){
        return run(()->{
            io.drive(new Translation2d(
                translationX.getAsDouble() * io.getMaximumVelocity(),
                translationY.getAsDouble() * io.getMaximumVelocity()),
                angularRotationX.getAsDouble() * io.getMaximumAngularVelocity(),
                true,
                false);
          });
    }


    public Command getAutonomousCommand(String name, boolean setOdomToStart){
        return io.getAutonomousCommand(name, setOdomToStart);
    }

}
