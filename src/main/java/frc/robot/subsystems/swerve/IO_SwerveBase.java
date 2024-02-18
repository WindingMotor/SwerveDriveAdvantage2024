// Written by WindingMotor, 2024, Crescendo

package frc.robot.subsystems.swerve;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.vision.SUB_Vision;

public interface IO_SwerveBase{

    @AutoLog
    public static class SwerveInputs{
        public Pose2d pose = new Pose2d();
        public Rotation2d yaw = new Rotation2d();
        public Rotation2d odometryHeading = new Rotation2d();
        public SwerveModulePosition[] positions = new SwerveModulePosition[4];
        public Pose2d estimatedLeftPose = new Pose2d();
        public Pose2d estimatedRightPose = new Pose2d();
        
        public SwerveModuleState[] realStates = new SwerveModuleState[4];
        public double[] desiredStates = new double[4];
    }

    /**
     * Updates the inputs with the current values.
     * @param inputs The inputs to update.
    */
    void updateInputs(SwerveInputs inputs);

    void drive(Translation2d translation, double rotation, boolean isFieldRelative, boolean isOpenLoop);

    double getMaximumVelocity();

    double getMaximumAngularVelocity();

    void updateEstimations(SUB_Vision vision);

    void updateOdometry();

    void setupPathPlanner(SUB_Swerve subsystem);

    void resetOdometry(Pose2d pose);

    Rotation2d getHeading();



}
