package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.wmlib2j.command.SwerveJoystick;
import frc.robot.wmlib2j.sensor.IO_GyroNavx;
import frc.robot.wmlib2j.swerve.IO_ModuleReal;
import frc.robot.wmlib2j.swerve.Swerve;
import frc.robot.wmlib2j.vision.IO_VisionReal;
import frc.robot.wmlib2j.vision.Vision;

public class RobotContainer {

    private final CommandXboxController driverController = new CommandXboxController(3);


    private Vision vision = new Vision(
        new IO_VisionReal()
    );
    
    private final Swerve swerve = new Swerve(
            new IO_ModuleReal(Constants.ModuleSettings.FRONT_LEFT),
            new IO_ModuleReal(Constants.ModuleSettings.FRONT_RIGHT),
            new IO_ModuleReal(Constants.ModuleSettings.BACK_LEFT),
            new IO_ModuleReal(Constants.ModuleSettings.BACK_RIGHT),
            new IO_GyroNavx()
    );

    
    public RobotContainer(){

        configureBindings();

        /* 
        swerve.setDefaultCommand(new SwerveJoystick(
                () -> driverBindings.xInverted ? -driverController.getRawAxis(driverBindings.xInput) : driverController.getRawAxis(driverBindings.xInput),
                () -> driverBindings.yInverted ? -driverController.getRawAxis(driverBindings.yInput) : driverController.getRawAxis(driverBindings.yInput),
                () -> driverBindings.rInverted ? -driverController.getRawAxis(driverBindings.rInput) : driverController.getRawAxis(driverBindings.rInput),
                () -> false,
                swerve
        ));
        */

        swerve.setDefaultCommand(new SwerveJoystick(
                () -> driverController.getRawAxis(1) ,
                () -> driverController.getRawAxis(0) ,
                () -> driverController.getRawAxis(3) ,
                () -> true,
                swerve
        ));
      
    }

    private void configureBindings(){
        // Add any additional bindings configuration here
    }

    public Command getAutonomousCommand(){
        return new PrintCommand("Test");
    }

    public void resetSwerveEncoders(){
        swerve.resetModuleEncoders();
    }

  
}
