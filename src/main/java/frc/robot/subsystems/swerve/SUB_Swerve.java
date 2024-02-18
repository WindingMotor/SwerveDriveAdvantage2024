// Written by WindingMotor, 2024, Crescendo

package frc.robot.subsystems.swerve;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.vision.SUB_Vision;

public class SUB_Swerve extends SubsystemBase{

    private final IO_SwerveBase io;

    private final SwerveInputsAutoLogged inputs = new SwerveInputsAutoLogged();

    private final SUB_Vision vision;

    public SUB_Swerve(IO_SwerveBase io, SUB_Vision vision){
        this.io = io;
        this.vision = vision;

        // Configure PathPlanner
        io.setupPathPlanner(this);
    }

    public void periodic(){

        
        if(Constants.SWERVE_VISION_ODOMETRY_ENABLED) {
            // Updates the pose estimations 
            io.updateEstimations(vision);      
        }
        // ISAAC: I MOVED THE PREVIOUS CODE HERE TO THE updateEstimations METHOD!

        io.updateOdometry();
        io.updateInputs(inputs);
        Logger.processInputs("Swerve", inputs);
    }

    /**
     * Drives the robot, in field-relative, based of the specified inputs.
     * @param  translationX      A supplier for the X translation
     * @param  translationY      A supplier for the Y translation
     * @param  angularRotationX  A supplier for the angular rotation
     * @return                   The command for driving the swerve drive
    */
    public Command driveJoystick(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX){
        return run(()->{
            io.drive(new Translation2d(
                translationX.getAsDouble() * io.getMaximumVelocity(),
                translationY.getAsDouble() * io.getMaximumVelocity()),
                angularRotationX.getAsDouble() * io.getMaximumAngularVelocity(),
                true,
                false);
          });
    }

    /**
     * Drives the robot to the specified pose.
     * @param  pose  The target pose to drive to
     * @return       The command to run to drive to the pose.
     */
    public Command driveToPose(Pose2d pose){

        DriverStation.reportWarning("[init] [driveToPose] Driving to " + pose.toString() + "", false);
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
            pose,
            new PathConstraints(
                1,
                1.5,
                Units.degreesToRadians(350), // 540
                Units.degreesToRadians(540) // 720
            )
        );
        return pathfindingCommand;
    }

    /**
     * Drives the robot to the speaker depending on the alliance.
     * @return       The command to run to drive to the pose.
     */
    public Command driveToSpeaker(){
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if(alliance.isPresent() && alliance.get() == Alliance.Blue){
            return driveToPose(Constants.Auto.ScoringPoses.BLU_SPEAKER.pose);
        }else if(alliance.isPresent() && alliance.get() == Alliance.Red){
            return driveToPose(Constants.Auto.ScoringPoses.RED_SPEAKER.pose);
        }
        return new PrintCommand("[error] [driveToSpeaker] No Alliance Detected!");
    }

    /**
     * Drives the robot to the amp depending on the alliance.
     * @return       The command to run to drive to the pose.
     */
    public Command driveToAmp(){
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if(alliance.isPresent() && alliance.get() == Alliance.Blue){
            return driveToPose(Constants.Auto.ScoringPoses.BLU_AMP.pose);
        }else if(alliance.isPresent() && alliance.get() == Alliance.Red){
            return driveToPose(Constants.Auto.ScoringPoses.RED_AMP.pose);
        }
        return new PrintCommand("[error] [driveToAmp] 'No Alliance Detected!");
    }

    /**
     * Drives the robot on the path specified.
     * @param  name  The name of the path to follow
     * @param  setOdomToStart  Whether or not to reset the odometry to the start of the path
     * @return       The command to run to drive the path
     */
    public Command drivePath(String name, boolean setOdomToStart){
        PathPlannerPath path = PathPlannerPath.fromPathFile(name);
        if(setOdomToStart){ 
            io.resetOdometry(
                new Pose2d(path.getPoint(0).position, io.getHeading())
            );
        }
        DriverStation.reportWarning("[init] [drivePath] Driving on path " + name + " and setOdomToStart is " + setOdomToStart + "", false);
        return AutoBuilder.followPath(path);
    }
}
