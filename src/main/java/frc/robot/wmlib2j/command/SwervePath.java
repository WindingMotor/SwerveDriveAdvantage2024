
package frc.robot.wmlib2j.command;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.wmlib2j.swerve.Swerve;
/**
 * Controls the swerve drive with joystick axes.
 * Creates chassis speeds and sends them to the swerve subsystem.
*/
public class SwervePath extends SequentialCommandGroup{

    public SwervePath(Swerve swerve, String pathName, boolean isFirstPath){

        PathPlannerPath loadedPath = PathPlannerPath.fromPathFile(pathName);

        // This command requires the swerve drive subsystem.
        addRequirements(swerve);

        if(isFirstPath){
            addCommands(null);
        }
    }

}