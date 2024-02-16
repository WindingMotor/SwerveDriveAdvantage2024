// Written by WindingMotor, 2024, Crescendo

package frc.robot.auto;
import org.littletonrobotics.junction.Logger;

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
    
        autoSelector = AutoBuilder.buildAutoChooser("A1_1D_SM");
        
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
