package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.States.SuperStructureState;
import frc.robot.beluga.SUB_SuperStructure;
import frc.robot.beluga.arm.IO_ArmReal;
import frc.robot.beluga.arm.SUB_Arm;
import frc.robot.beluga.shooter.IO_ShooterReal;
import frc.robot.beluga.shooter.SUB_Shooter;
import frc.robot.yagsl.IO_SwerveReal;
import frc.robot.yagsl.SUB_Swerve;

public class RobotContainer {

    private final CommandXboxController driverController = new CommandXboxController(3);
    private final CommandXboxController operatorController = new CommandXboxController(4);

/* 
    private final Vision vision = new Vision(
        new IO_VisionReal()
    );
    */
    
    private final SUB_Swerve swerve = new SUB_Swerve(
        new IO_SwerveReal()
    );

    private final SUB_Arm arm = new SUB_Arm(
        new IO_ArmReal()
    );

    private final SUB_Shooter shooter = new SUB_Shooter(
        new IO_ShooterReal()
    );

    private final SUB_SuperStructure superstructure = new SUB_SuperStructure(
        arm, shooter
    );

    public RobotContainer(){

        configureBindings();

    
        
        swerve.setDefaultCommand( 
            swerve.driveSwerve(() -> -driverController.getRawAxis(1) , () -> driverController.getRawAxis(0) , () -> driverController.getRawAxis(3))
        );
        

    }

    private void configureBindings(){

        operatorController.b().onTrue(
            superstructure.setState(SuperStructureState.OFF)
        );

        operatorController.a().onTrue(
            superstructure.setState(SuperStructureState.DEMO)
        );
        
    }

    public Command getAutonomousCommand(){
        return swerve.getAutonomousCommand("Test", true);
    }
  
}
