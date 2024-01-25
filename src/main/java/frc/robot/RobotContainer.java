package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.wmlib2j.vision.IO_VisionReal;
import frc.robot.wmlib2j.vision.Vision;
import frc.robot.yagsl.IO_SwerveReal;
import frc.robot.yagsl.Swerve;

public class RobotContainer {

    private final CommandXboxController driverController = new CommandXboxController(3);
/* 
    private final Vision vision = new Vision(
        new IO_VisionReal()
    );
    */
    
    private final Swerve swerve = new Swerve(
        new IO_SwerveReal()
    );

    public RobotContainer(){

        configureBindings();

    
        
        swerve.setDefaultCommand( 
            swerve.driveSwerve(() -> driverController.getRawAxis(5) , () -> -driverController.getRawAxis(4) , () -> driverController.getRawAxis(0))
        );
        

    }

    private void configureBindings(){
        // Add any additional bindings configuration here
    }

    public Command getAutonomousCommand(){
        return swerve.getAutonomousCommand("Test", true);
    }
  
}
