// Written by WindingMotor, 2024, Crescendo

package frc.robot.subsystems.swerve;
import java.io.File;
import java.io.IOException;
import org.photonvision.EstimatedRobotPose;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
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
        inputs.states = swerveDrive.getStates();
        inputs.positions = swerveDrive.getModulePositions();
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

    // test

    @Override
    public Rotation2d getHeading(){
        // return swerveDrive.getYaw();
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

    /** See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double)}. */
    public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
        swerveDrive.addVisionMeasurement(visionMeasurement, timestampSeconds);
    }

    /** See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}. */
    public void addVisionMeasurement(
            Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        swerveDrive.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
    }

    @Override
    public void updateOdometry(){
        swerveDrive.updateOdometry();
    }

}
