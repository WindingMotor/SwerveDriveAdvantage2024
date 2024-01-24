
package frc.robot.yagsl;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;

public interface IO_SwerveBase{

    @AutoLog
    public static class SwerveInputs{
        public Pose2d pose = new Pose2d();
        public Rotation2d yaw = new Rotation2d();
        public Rotation2d odometryHeading = new Rotation2d();
        public SwerveModuleState[] states = new SwerveModuleState[4];
        public SwerveModulePosition[] positions = new SwerveModulePosition[4];
    }

    /**
     * Updates the inputs with the current values.
     * @param inputs The inputs to update.
    */
    void updateInputs(SwerveInputs inputs);

    void drive(Translation2d translation, double rotation, boolean isFieldRelative, boolean isOpenLoop);

    double getMaximumVelocity();

    double getMaximumAngularVelocity();

    void addVisionMeasurement(EstimatedRobotPose pose);

    void updateOdometry();

    void setupPathPlanner(Swerve subsystem);

    Command getAutonomousCommand(String pathName, boolean resetPose);

}
