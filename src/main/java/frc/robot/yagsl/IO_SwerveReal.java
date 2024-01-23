
package frc.robot.yagsl;

import java.io.File;
import java.io.IOException;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * Represents a real implementation of the shooter.
*/
public class IO_SwerveReal implements IO_SwerveBase{

    private static double maxSpeed = Units.feetToMeters(4.5);
    private static File jsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
    private static SwerveDrive swerveDrive;

    public IO_SwerveReal(){

        // Initialize the swerve drive based of the json file
        try{
            swerveDrive = new SwerveParser(jsonDirectory).createSwerveDrive(maxSpeed);
        }catch(IOException e){
            e.printStackTrace();
        }

        // Set the telemetry verbosity to high for debugging
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    }

    /**
     * Updates the inputs with the current values.
     * @param inputs The inputs to update.
    */
    @Override
    public void updateInputs(SwerveInputs inputs){
        inputs.pose = swerveDrive.getPose();
        inputs.yaw = swerveDrive.getYaw();
        inputs.odometryHeading = swerveDrive.getOdometryHeading();
        inputs.states = swerveDrive.getStates();
        inputs.positions = swerveDrive.getModulePositions();
    }

    @Override
    public void drive(Translation2d translation, double rotation, boolean isFieldRelative, boolean isOpenLoop){
        swerveDrive.drive(translation, rotation, isFieldRelative, isOpenLoop);
    }

    @Override
    public double getMaximumVelocity(){
        return swerveDrive.getMaximumVelocity();
    }

    @Override
    public double getMaximumAngularVelocity(){
        return swerveDrive.getMaximumAngularVelocity();
    }

    @Override
    public void addVisionMeasurement(EstimatedRobotPose pose){
        swerveDrive.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
    }

    @Override
    public void updateOdometry(){
        swerveDrive.updateOdometry();
    }

}
