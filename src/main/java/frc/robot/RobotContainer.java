package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.wmlib2j.sensor.IO_GyroNavx;

import frc.robot.wmlib2j.vision.IO_VisionReal;
import frc.robot.wmlib2j.vision.Vision;

public class RobotContainer {

    private final CommandXboxController driverController = new CommandXboxController(3);

    private Vision vision = new Vision(
        new IO_VisionReal()
    );
    
    public RobotContainer(){

        configureBindings();

    }

    private void configureBindings(){
        // Add any additional bindings configuration here
    }

    public Command getAutonomousCommand(){
        return new PrintCommand("Test");
    }
  
}
