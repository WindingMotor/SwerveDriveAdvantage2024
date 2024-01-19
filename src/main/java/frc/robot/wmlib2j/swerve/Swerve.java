// Written by WindingMotor as part of the wmlib2j library.

package frc.robot.wmlib2j.swerve;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.wmlib2j.sensor.IO_GyroBase;

import java.io.IOException;
import java.sql.Driver;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Represents a Swerve drive subsystem.
 * Responsible for controlling a the swerve drive,
 * which consists of four swerve modules. It provides methods for controlling
 * the movement and rotation of the robot.
*/
public class Swerve extends SubsystemBase{

    // Gyroscope IO
    private final IO_GyroBase gyroIO;
    public final IO_GyroBase.GyroIOInputs gyroInputs = new IO_GyroBase.GyroIOInputs();

    // Swerve modules
    private final Module frontLeftModule;
    private final Module frontRightModule;
    private final Module backLeftModule;
    private final Module backRightModule;

    // Setpoint speeds for the chassis
    private ChassisSpeeds setpointSpeeds = new ChassisSpeeds();

    // Odometry for the swerve drive
    private final SwerveDriveOdometry traditionalOdometry;
    private final SwerveDrivePoseEstimator poseEstimator;


    // An array of the 4 swerve modules
    private final Module modules[] = new Module[4];


    /**
     * Constructor for the Swerve class.
     * @param frontLeftIO The IO for the front left swerve module.
     * @param frontRightIO The IO for the front right swerve module.
     * @param backLeftIO The IO for the back left swerve module.
     * @param backRightIO The IO for the back right swerve module.
     * @param gyroIO The IO for the gyroscope.
     * @param vision The vision subsystem.
    */
    public Swerve(IO_ModuleBase frontLeftIO, IO_ModuleBase frontRightIO, IO_ModuleBase backLeftIO, IO_ModuleBase backRightIO, IO_GyroBase gyroIO){
        this.gyroIO = gyroIO;

        // Initialize each swerve module
        this.frontLeftModule = new Module(frontLeftIO, Constants.ModuleSettings.FRONT_LEFT);
        this.frontRightModule = new Module(frontRightIO, Constants.ModuleSettings.FRONT_RIGHT);
        this.backLeftModule = new Module(backLeftIO, Constants.ModuleSettings.BACK_LEFT);
        this.backRightModule = new Module(backRightIO, Constants.ModuleSettings.BACK_RIGHT);

        // Add each swerve module to the array
        this.modules[0] = this.frontLeftModule;
        this.modules[1] = this.frontRightModule;
        this.modules[2] = this.backLeftModule;
        this.modules[3] = this.backRightModule;

        resetModuleEncoders();

        // Initialize traditional odometry and the pose estimator
        this.traditionalOdometry = new SwerveDriveOdometry(Constants.Kinematics.KINEMATICS, gyroInputs.yawPosition, getSwerveModulePositions());

        this.poseEstimator = new SwerveDrivePoseEstimator(Constants.Kinematics.KINEMATICS, gyroInputs.yawPosition,
            getSwerveModulePositions(), traditionalOdometry.getPoseMeters());

        // Configure default settings for the AutoBuilder
        configureAutoBuilder();
    }

    /**
     * Executes the periodic tasks for the swerve drive. Updates and logs the swerve modules.
    */
    @Override
    public void periodic(){

        // Log and process gyroscope inputs
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Gyroscope", gyroInputs);

        // Update traditional odometry
        traditionalOdometry.update(gyroInputs.yawPosition, getSwerveModulePositions());

        // Update the pose estimator
        poseEstimator.update(getRobotVisionRotation(), getSwerveModulePositions())

        // Execute each swerve modules periodic method
        for (Module module : modules){
            module.periodic();
        }

        // Update the swerve module states and return them as an array
        SwerveModuleState[] states = updateSwerveStates();

        // Log setpoint states
        Logger.recordOutput("SwerveStates/setpoints", states);

        // Fetch and log the current states
        SwerveModuleState[] actualStates = new SwerveModuleState[4];
        for (int i = 0; i < actualStates.length; i++) {
            actualStates[i] = modules[i].getModuleState();
        }
        Logger.recordOutput("SwerveStates/Measured", actualStates);

        // Log the current robot pose in 3D, 2D, and 2D traditional
        Logger.recordOutput("Odometry/estimatedPose", new Pose2d(traditionalOdometry.getPoseMeters().getTranslation(), getRobotVisionRotation()));
        Logger.recordOutput("Odometry/traditionalPose", new Pose2d(traditionalOdometry.getPoseMeters().getTranslation(), new Rotation2d(traditionalOdometry.getPoseMeters().getRotation().getRadians())));
    }

    /**
     * Updates the swerve module states based on the current setpoint speeds.
     * @return An array of SwerveModuleState objects representing the updated module states.
    */
    public SwerveModuleState[] updateSwerveStates(){
        // Get the swerve module states from kinematics method using current setpointSpeeds
        SwerveModuleState[] states = Constants.Kinematics.KINEMATICS.toSwerveModuleStates(setpointSpeeds);

        // Desaturate the states to make every turn position and velocity possible
        SwerveDriveKinematics.desaturateWheelSpeeds(states, 4.54);

        // Set each swerve module with its respective state
        frontLeftModule.runWithState(states[0]);
        frontRightModule.runWithState(states[1]);
        backLeftModule.runWithState(states[2]);
        backRightModule.runWithState(states[3]);

        return states;
    }

    /**
     * Sets the robot swerve setpoint to the desired chassis speeds.
     * @param newSpeeds The new chassis speeds.
    */
    public void updateSpeedSetpoint(ChassisSpeeds newSpeeds){
        setpointSpeeds = newSpeeds;
    }

    /**
     * Stops the swerve modules.
    */
    public void stop(){
        updateSpeedSetpoint(new ChassisSpeeds());
    }

    /**
     * Resets the encoders of all swerve modules.
    */
    public void resetModuleEncoders(){
        for(Module module : modules){
            module.resetEncoders();
        }
    }

    /**
     * Retrieves the positions of all swerve modules.
     * @return An array of SwerveModulePosition objects representing the positions of the swerve modules.
    */
    public SwerveModulePosition[] getSwerveModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(int i = 0; i < positions.length; i++){
            positions[i] = modules[i].getModulePosition();
        }
        return positions;
    }

    /**
     * Returns the estimated pose of the robot.
     * @return The estimated pose of the robot in meters and radians.
    */
    public Pose2d getRobotVisionPose(){
        //return new Pose2d(traditionalOdometry.getPoseMeters().getTranslation(), getRobotVisionRotation());
        return new Pose2d(
            vision.
        )
    }

    /**
     * Returns the estimated rotation of the robot.
     * @return The estimated rotation of the robot in radians.
    */
    public Rotation2d getRobotVisionRotation(){
        return gyroInputs.yawPosition;
    }

    /**
     * Retrieves the current pose of the robot.
     * @return  The current pose of the robot as a Pose2d object
    */
    public Pose2d getRobotPose(){
        return traditionalOdometry.getPoseMeters();
    }

    /**
     * Resets the pose of the robot to the specified new pose.
     * @param  newPose  The new pose2d to set for the robot
    */
    public void resetPose(Pose2d newPose){
        traditionalOdometry.resetPosition(newPose.getRotation(), getSwerveModulePositions(), newPose);
    }

    /**
     * Retrieves the setpoint speeds for the chassis.
     * @return The setpoint speeds for the chassis.
    */
    public ChassisSpeeds getSetpointSpeeds(){
        return setpointSpeeds;
    }

    // Pathplanner AutoBuilder configuration
    private void configureAutoBuilder(){
        AutoBuilder.configureHolonomic(
            this::getRobotPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getSetpointSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::updateSpeedSetpoint, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            4.5, // Max module speed, in m/s
            0.4, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ), () -> {

        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if(alliance.isPresent()){
            return alliance.get() == DriverStation.Alliance.Red;
        }
            return false;
        },
        this // Reference to this subsystem to set requirements
        );
    }

}
