// Written by WindingMotor, 2024, Crescendo

package frc.robot.subsystems.swerve;
import java.io.File;
import java.io.IOException;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import frc.robot.Constants.Vision.Camera;
import frc.robot.subsystems.vision.SUB_Vision;
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

    // Local pose estimates that will be updated and sent to updateInputs when updateEstimations is called
    private Pose2d localEstimatedLeftPose = new Pose2d();
    private Pose2d localEstimatedRightPose = new Pose2d();

    public IO_SwerveReal(){

        // Initialize the swerve drive based of the json file
        try{
            swerveDrive = new SwerveParser(jsonDirectory).createSwerveDrive(maxSpeed);
        }catch(IOException e){
            e.printStackTrace();
        }

        // Set the telemetry verbosity to high for debugging
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        // Should only be enabled when controlling the robot via angle
        swerveDrive.setHeadingCorrection(false);

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
        inputs.positions = swerveDrive.getModulePositions();
        inputs.estimatedLeftPose = localEstimatedLeftPose;
        inputs.estimatedRightPose = localEstimatedRightPose;
        inputs.realStates = swerveDrive.getStates();
        inputs.desiredStates = SwerveDriveTelemetry.desiredStates;
    }

    public void setupPathPlanner(SUB_Swerve subsystem){
        AutoBuilder.configureHolonomic(
            this::getPose, // Gets current robot pose
            this::resetOdometry, // Resets robot odometry if path has starter pose
            this::getRobotVelocity, // Gets chassis speed in robot relative
            this::setChassisSpeeds, // Drives the robot in robot realative chassis speeds
            new HolonomicPathFollowerConfig(
                Constants.Auto.AUTO_PID, // Path follower PID constants

                new PIDConstants(swerveDrive.swerveController.config.headingPIDF.p, // Rotation PID
                    swerveDrive.swerveController.config.headingPIDF.i,
                    swerveDrive.swerveController.config.headingPIDF.d),
                    Constants.Auto.MAX_MODULE_SPEED_MPS,
                swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(), // Drive base radius in meters

                new ReplanningConfig() // Replanning config see docs
            ),()->{
                // Auto path flipper for allaince color, always make baths on blue side
                var alliance = DriverStation.getAlliance();
                return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
            }, subsystem 
        );
    }

    private Pose2d getPose(){
        return swerveDrive.getPose();
    }

    @Override
    public void resetOdometry(Pose2d pose){
        swerveDrive.resetOdometry(pose);
    }

    private ChassisSpeeds getRobotVelocity(){
        return swerveDrive.getRobotVelocity();
    }

    private void setChassisSpeeds(ChassisSpeeds chassisSpeeds){
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    @Override
    public Rotation2d getHeading(){
        return getPose().getRotation();
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

    /**
     * Updates the swerve drive's estimations based on vision measurements from the specified camera.
     * Only adds the measurement if it's not ambiguous.
     * @param vision The vision subsystem providing the measurements.
    */
    @Override
    public void updateEstimations(SUB_Vision vision){

        // Get the estimated global pose from the left camera
        var leftVisionEst = vision.getEstimatedGlobalPose(Camera.LEFT_CAMERA);

        if (leftVisionEst != null){
            leftVisionEst.ifPresent(est ->{

                // Check if the pose estimation is ambiguous
                boolean isAmbiguous = est.targetsUsed.stream()
                        .anyMatch(tar -> tar.getPoseAmbiguity() >  0.2);

                // If the pose is not ambiguous, add the vision measurement to the swerve drive
                if (!isAmbiguous){
                    var estPose = est.estimatedPose.toPose2d();
                    var estStdDevs = vision.getEstimationStdDevs(estPose, Camera.LEFT_CAMERA);
                    swerveDrive.addVisionMeasurement(estPose, est.timestampSeconds, estStdDevs);
                    localEstimatedLeftPose = estPose;
                }
            });
        }

        // Get the estimated global pose from the right camera
        var rightVisionEst = vision.getEstimatedGlobalPose(Camera.RIGHT_CAMERA);

        if (rightVisionEst != null){
            rightVisionEst.ifPresent(est -> {

                // Check if the pose estimation is ambiguous
                boolean isAmbiguous = est.targetsUsed.stream()
                        .anyMatch(tar -> tar.getPoseAmbiguity() >  0.2);

                // If the pose is not ambiguous, add the vision measurement to the swerve drive
                if (!isAmbiguous){
                    var estPose = est.estimatedPose.toPose2d();
                    var estStdDevs = vision.getEstimationStdDevs(estPose, Camera.RIGHT_CAMERA);
                    swerveDrive.addVisionMeasurement(estPose, est.timestampSeconds, estStdDevs);
                    localEstimatedRightPose = estPose;
                }
            });
        }
    }

    @Override
    public void updateOdometry(){
        swerveDrive.updateOdometry();
    }
    

}
