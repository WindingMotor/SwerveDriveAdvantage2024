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

    private double autosLoaded = 0;

    public AutoSelector(){

        shuffleboardTab = Shuffleboard.getTab("Auto Selector");
    
        autoSelector = new SendableChooser<Command>();

        autoSelector.setDefaultOption("A3_3D_SM", AutoBuilder.buildAuto("A3_3D_SM"));
        
        AutoBuilder.getAllAutoNames().forEach( name -> {
            autoSelector.addOption(name, AutoBuilder.buildAuto(name));
            DriverStation.reportError("[init] [auto] " + name, false);
            autosLoaded += 1;
        });

        Logger.recordMetadata("Autos Loaded", autosLoaded + "");
        
        shuffleboardTab.add(autoSelector);
    }    

    public Command getSelectedAuto(){
        return autoSelector.getSelected();
    }
}
