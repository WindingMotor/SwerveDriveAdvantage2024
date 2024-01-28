
 package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.States.SuperStructureStatePosition;
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

    private final CommandJoystick driverSim = new CommandJoystick(0);

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
            //swerve.driveSwerve(() -> -driverController.getRawAxis(1) , () -> driverController.getRawAxis(0) , () -> driverController.getRawAxis(3))
             swerve.driveJoystick(() -> -driverSim.getRawAxis(4) , () -> -driverSim.getRawAxis(3) , () -> -driverSim.getRawAxis(0))
        );
    
    }

    private void configureBindings(){


        // Turn off the superstructure, disables all motors. Use in case of an emergency.
        operatorController.b().and(operatorController.a()).onTrue(
            superstructure.setStatePosition(SuperStructureStatePosition.OFF)
        );

        // Sets the superstructure to the idle mode.
        operatorController.b().onTrue(
            superstructure.setStatePosition(SuperStructureStatePosition.IDLE)
        );

        // Sets the superstructure to the speaker mode for manual scoring.
        operatorController.a().onTrue(
            superstructure.setStatePosition(SuperStructureStatePosition.DEMO)
        );

        // Sets the superstructure to the target mode for calculating target distance and scoring autonomously.
        operatorController.a().onTrue(
            superstructure.setStatePosition(SuperStructureStatePosition.DEMO)
        );

        driverSim.button(1).onTrue(swerve.driveToPose(Constants.Auto.Poses.BLU_SPEAKER));
        
    }

    public Command getAutonomousCommand(){
        return swerve.getAutonomousCommand("Test", true);
    }
    
  }
