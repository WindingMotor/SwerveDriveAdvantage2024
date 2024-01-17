package frc.robot.wmlib2j.swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.wmlib2j.sensor.IO_GyroBase;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;

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
    private final SwerveDriveOdometry odometry;
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

        // Initialize the odometry and pose estimator
        this.odometry = new SwerveDriveOdometry(Constants.Kinematics.KINEMATICS, gyroInputs.yawPosition, getSwerveModulePositions());
        this.poseEstimator = new SwerveDrivePoseEstimator(Constants.Kinematics.KINEMATICS, getRobotEstimatedRotation(), getSwerveModulePositions(), getRobotEstimatedPose());



    }


    /**
     * Executes the periodic tasks for the swerve drive. Updates and logs the swerve modules.
    */
    @Override
    public void periodic(){

        // Log and process gyroscope inputs
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Gyroscope", gyroInputs);

        odometry.update(gyroInputs.yawPosition, getSwerveModulePositions());

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
        Logger.recordOutput("Odometry/estimatedPose", new Pose2d(odometry.getPoseMeters().getTranslation(), getRobotEstimatedRotation()));
        Logger.recordOutput("Odometry/traditionalPose", new Pose2d(odometry.getPoseMeters().getTranslation(), new Rotation2d(odometry.getPoseMeters().getRotation().getRadians())));

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
    public void runWithSpeeds(ChassisSpeeds newSpeeds){
        setpointSpeeds = newSpeeds;
    }

    /**
     * Stops the swerve modules.
    */
    public void stop(){
        runWithSpeeds(new ChassisSpeeds());
    }

    public void resetModuleEncoders(){
        frontLeftModule.resetEncoders();
        frontRightModule.resetEncoders();
        backLeftModule.resetEncoders();
        backRightModule.resetEncoders();
    }

    /**
     * Retrieves the positions of all swerve modules.
     * @return An array of SwerveModulePosition objects representing the positions of the swerve modules.
     */
    public SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < positions.length; i++) {
            positions[i] = modules[i].getModulePosition();
        }
        return positions;
    }

    /**
     * Returns the estimated pose of the robot.
     * @return The estimated pose of the robot in meters and radians.
    */
    public Pose2d getRobotEstimatedPose(){
        return new Pose2d(odometry.getPoseMeters().getTranslation(), getRobotEstimatedRotation());
    }

    /**
     * Returns the estimated rotation of the robot.
     * @return The estimated rotation of the robot in radians.
    */
    public Rotation2d getRobotEstimatedRotation(){
        return gyroInputs.yawPosition;

    
    }


}
