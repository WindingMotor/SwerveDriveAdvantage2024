// Written by WindingMotor, 2024, Crescendo

package frc.robot.auto;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoSelector{
    
    private final ShuffleboardTab shuffleboardTab;

    private SendableChooser<Command> autoSelector;

    public AutoSelector(){

        shuffleboardTab = Shuffleboard.getTab("Auto Selector");
    
        // Build the auto selector from PathPlanner auto files and set the default auto to A1_D1_SM
        autoSelector = AutoBuilder.buildAutoChooser("A1_D1_SM");

        // Not sure why this is needed, the auto selector should apply all PathPlannerAuto's options by default
        //autoSelector.addOption("A1_D1_SM", getSelectedAuto());
        
        shuffleboardTab.add(autoSelector);
    }    

    /**
     * Gets the command of the selected auto.
     * @command The command of the selected auto
    */
    public Command getSelectedAuto(){
        return autoSelector.getSelected();
    }

    /**
     * Gets the starting pose of the selected auto.
     * @return The starting pose of the selected auto
    */
    public Pose2d getStartingPose(){
        return PathPlannerAuto.getStaringPoseFromAutoFile(autoSelector.getSelected().getName());
    }

}
