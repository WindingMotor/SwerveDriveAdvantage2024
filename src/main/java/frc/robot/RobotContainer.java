
package frc.robot;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.States.ShooterMode;
import frc.robot.auto.AutoSelector;
import frc.robot.auto.CommandRegistrar;
import frc.robot.commands.CMDGR_DrivePose;
import frc.robot.commands.CMDGR_Shoot;
import frc.robot.commands.CMD_Eject;
import frc.robot.commands.CMD_Idle;
import frc.robot.commands.CMD_Intake;
import frc.robot.subsystems.arm.IO_ArmReal;
import frc.robot.subsystems.arm.SUB_Arm;
import frc.robot.subsystems.conveyor.CMD_IndexerManual;
import frc.robot.subsystems.conveyor.IO_ConveyorReal;
import frc.robot.subsystems.conveyor.SUB_Conveyor;
import frc.robot.subsystems.shooter.IO_ShooterReal;
import frc.robot.subsystems.shooter.SUB_Shooter;
import frc.robot.subsystems.swerve.IO_SwerveReal;
import frc.robot.subsystems.swerve.SUB_Swerve;
import frc.robot.subsystems.vision.IO_VisionReal;
import frc.robot.subsystems.vision.SUB_Vision;
import frc.robot.util.AddressableLedStrip;
import swervelib.telemetry.SwerveDriveTelemetry;

public class RobotContainer{

    private final CommandXboxController driverController = new CommandXboxController(3);
    private final CommandXboxController operatorController = new CommandXboxController(4);

    private final SUB_Vision vision = new SUB_Vision( new IO_VisionReal() );
    
    // All methods using these subsystems should be called in this order -> conveyor, arm, shooter
    private final SUB_Conveyor conveyor = new SUB_Conveyor( new IO_ConveyorReal() );

    private final SUB_Arm arm = new SUB_Arm( new IO_ArmReal() );

    private final SUB_Shooter shooter = new SUB_Shooter( new IO_ShooterReal() );

    private final AddressableLedStrip led = new AddressableLedStrip(0, 71);

    private final SUB_Swerve swerve = new SUB_Swerve( new IO_SwerveReal(), vision );  

    private final CommandRegistrar commandRegistrar = new CommandRegistrar(vision, swerve, conveyor, arm, shooter, led);

    private final AutoSelector autoSelector;


    public RobotContainer(){
        
        commandRegistrar.register();

        autoSelector = new AutoSelector();
        

        configureBindings();

        logMetadata();
    }

    /**
     * Configure the bindings for the controller buttons to specific commands.
    */
    private void configureBindings(){

        // Default drive command
        swerve.setDefaultCommand( 
            swerve.driveJoystick(() -> driverController.getRawAxis(1) , () -> -driverController.getRawAxis(0) , () -> -driverController.getRawAxis(3))
        );

        // Shoot command
        operatorController.x().onTrue(new CMDGR_Shoot(conveyor, arm, shooter, vision, led,
            ShooterMode.SPEAKER, () -> operatorController.b().getAsBoolean(), () -> operatorController.x().getAsBoolean()
        ));
        
        // Amp command
        operatorController.y().onTrue(new CMDGR_Shoot(conveyor, arm, shooter, vision, led,
            ShooterMode.AMP, () -> operatorController.b().getAsBoolean(), () -> operatorController.y().getAsBoolean()
        ));

        // Intake command
        operatorController.a().onTrue(new CMD_Intake(conveyor, arm, () -> operatorController.b().getAsBoolean()));
        
        // Eject command
        operatorController.rightBumper().onTrue(new CMD_Eject(conveyor, arm, () -> operatorController.b().getAsBoolean()));

        // Idle command
        operatorController.b().onTrue(new CMD_Idle(conveyor, arm, shooter));

        // Drive to speaker command
        //operatorController.povRight().debounce(0.25).onTrue(new CMDGR_DrivePose(swerve, Constants.Auto.DriveScoringPoseState.SPEAKER, () -> operatorController.b().getAsBoolean()));

        // Drive to amp command
        //operatorController.povLeft().debounce(0.25).onTrue(new CMDGR_DrivePose(swerve, Constants.Auto.DriveScoringPoseState.AMP, () -> operatorController.b().getAsBoolean()));
    }

    private void logMetadata(){
        Logger.recordMetadata("Robot Mode",  Constants.CURRENT_MODE + "");
        DriverStation.reportWarning("[sys init] Robot Mode: " + Constants.CURRENT_MODE, false);

        Logger.recordMetadata("PID Test Mode",  Constants.PID_TEST_MODE + "");
        DriverStation.reportWarning("[sys init] PID Test Mode: " + Constants.PID_TEST_MODE, false);

        Logger.recordMetadata("Teleop Auto Drive",  Constants.TELEOP_AUTO_DRIVE_ENABLED + "");
        DriverStation.reportWarning("[sys init] Teleop Auto Drive: " + Constants.TELEOP_AUTO_DRIVE_ENABLED, false);
        
        Logger.recordMetadata("Swerve Drive Simulation", SwerveDriveTelemetry.isSimulation + "");
        DriverStation.reportWarning("[sys init] Swerve Drive Simulation: " + SwerveDriveTelemetry.isSimulation, false);
    }

    // TEST INSERT
    
    /**
     * Get the autonomous command.
     * @return The autonomous command
    */
    public Command getAutonomousCommand(){
        return autoSelector.getSelectedAuto();
    }
}