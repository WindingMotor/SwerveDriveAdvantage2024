package frc.robot.wmlib2j.swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.wmlib2j.sensor.IO_Gyro;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;


public class Swerve extends SubsystemBase {

    private final IO_Gyro gyroIO;
    public final IO_Gyro.GyroIOInputs gyroInputs = new IO_Gyro.GyroIOInputs();

    private final Module frontLeftModule;
    private final Module frontRightModule;
    private final Module backLeftModule;
    private final Module backRightModule;

    private ChassisSpeeds setpointSpeeds = new ChassisSpeeds();

    private final SwerveDriveOdometry traditionalOdometry;

    public Swerve(IO_ModuleBase frontLeftIO, IO_ModuleBase frontRightIO, IO_ModuleBase backLeftIO, IO_ModuleBase backRightIO, IO_Gyro gyroIO){
        this.gyroIO = gyroIO;

        this.frontLeftModule = new Module(frontLeftIO, Constants.ModuleSettings.FRONTLEFT);
        this.frontRightModule = new Module(frontRightIO, Constants.ModuleSettings.FRONTRIGHT);
        this.backLeftModule = new Module(backLeftIO, Constants.ModuleSettings.BACKLEFT);
        this.backRightModule = new Module(backRightIO, Constants.ModuleSettings.BACKRIGHT);


        this.traditionalOdometry = new SwerveDriveOdometry(Constants.Kinematics.KINEMATICS, gyroInputs.yawPosition, getSwerveModulePositions());
    }

    @Override
    public void periodic(){
        // Update gyroscope inputs.
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Gyroscope", gyroInputs);

        // Update modules periodic loop
        frontLeftModule.periodic();
        frontRightModule.periodic();
        backLeftModule.periodic();
        backRightModule.periodic();

        // Get the swerve module states from kinematics method using current setpointSpeeds
        SwerveModuleState[] states = Constants.Kinematics.KINEMATICS.toSwerveModuleStates(setpointSpeeds);

        // Desaturate the states, make every turn position and velocity possible.
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Kinematics.MAX_LINEAR_VELOCITY);

        // Set each swerve module with its respective state
        frontLeftModule.runWithState(states[0]);
        frontRightModule.runWithState(states[1]);
        backLeftModule.runWithState(states[2]);
        backRightModule.runWithState(states[3]);

        // Record the setpoint states
        Logger.recordOutput("SwerveStates/setpoints", states);

        // Record the current, "real", states
        SwerveModuleState[] measuredStates = new SwerveModuleState[4];
        measuredStates[0] = frontLeftModule.getModuleState();
        measuredStates[1] = frontRightModule.getModuleState();
        measuredStates[2] = backLeftModule.getModuleState();
        measuredStates[3] = backRightModule.getModuleState();

        Logger.recordOutput("SwerveStates/Measured", measuredStates);

        // Log the current robot pose in 3d, 2d, and 2d traditional
        Logger.recordOutput("Odometry/estimatedPose", new Pose2d(traditionalOdometry.getPoseMeters().getTranslation(), getRobotEstimatedRotation()));

        Logger.recordOutput("Odometry/traditionalPose", new Pose2d(traditionalOdometry.getPoseMeters().getTranslation(), new Rotation2d(traditionalOdometry.getPoseMeters().getRotation().getRadians())));
    }

    // Set the swerve setpoint to the desired chassis speeds
    public void runWithSpeeds(ChassisSpeeds newSpeeds){
        setpointSpeeds = newSpeeds;
    }

    // Stop swerve modules
    public void stop(){
        runWithSpeeds(new ChassisSpeeds());
    }

    public SwerveModulePosition[] getSwerveModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        positions[0] = frontLeftModule.getModulePosition();
        positions[1] = frontRightModule.getModulePosition();
        positions[2] = backLeftModule.getModulePosition();
        positions[3] = backRightModule.getModulePosition();
        return positions;
    }
    
    public Pose2d getRobotEstimatedPose(){
        return new Pose2d(traditionalOdometry.getPoseMeters().getTranslation(), getRobotEstimatedRotation());
    }

    public Rotation2d getRobotEstimatedRotation(){
        return gyroInputs.yawPosition; // REPLACE WITH POSE ESTIMATOR
    }

}
