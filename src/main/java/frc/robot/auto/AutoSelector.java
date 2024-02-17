// Written by WindingMotor, 2024, Crescendo

package frc.robot.auto;
import org.littletonrobotics.junction.Logger;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoSelector{
    
    private final ShuffleboardTab shuffleboardTab;

    private SendableChooser<Command> autoSelector;


    public AutoSelector(){

        shuffleboardTab = Shuffleboard.getTab("Auto Selector");
    
        autoSelector = AutoBuilder.buildAutoChooser("A1_D1_SM");
        autoSelector.addOption("A1_D1_SM", getSelectedAuto());
        
        shuffleboardTab.add(autoSelector);
    }    

    public Command getSelectedAuto(){
        return autoSelector.getSelected();
    }
}
