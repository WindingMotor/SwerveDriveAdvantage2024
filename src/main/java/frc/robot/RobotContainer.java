
 package frc.robot;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.States.ArmState;
import frc.robot.Constants.States.ShooterMode;
import frc.robot.commands.CMDGR_Shoot;
import frc.robot.commands.CMD_Arm;
import frc.robot.commands.CMD_Idle;
import frc.robot.commands.CMD_Intake;
import frc.robot.subsystems.arm.IO_ArmReal;
import frc.robot.subsystems.arm.SUB_Arm;
import frc.robot.subsystems.conveyor.CMD_IndexerManual;
import frc.robot.subsystems.conveyor.CMD_IndexerStop;
import frc.robot.subsystems.conveyor.IO_ConveyorReal;
import frc.robot.subsystems.conveyor.SUB_Conveyor;
import frc.robot.subsystems.shooter.IO_ShooterReal;
import frc.robot.subsystems.shooter.SUB_Shooter;
import frc.robot.subsystems.swerve.IO_SwerveReal;
import frc.robot.subsystems.swerve.SUB_Swerve;
import frc.robot.subsystems.vision.IO_VisionReal;
import frc.robot.subsystems.vision.SUB_Vision;
import frc.robot.util.AddressableLedStrip;

public class RobotContainer{

    private final CommandXboxController driverController = new CommandXboxController(3);
    private final CommandXboxController operatorController = new CommandXboxController(4);
    // private final CommandJoystick driverSim = new CommandJoystick(0);
    
    private final SUB_Vision vision = new SUB_Vision(
        new IO_VisionReal()
    );
    
    private final SUB_Swerve swerve = new SUB_Swerve(
        new IO_SwerveReal()
    );  

    private final SUB_Arm arm = new SUB_Arm(
        new IO_ArmReal()
    );

    private final SUB_Shooter shooter = new SUB_Shooter(
        new IO_ShooterReal()
    );

    private final SUB_Conveyor conveyor = new SUB_Conveyor(
        new IO_ConveyorReal()
    );

    private final AddressableLedStrip led = new AddressableLedStrip(0, 71);

    public RobotContainer(){

        configureBindings();
        
        swerve.setDefaultCommand( 
            swerve.driveJoystick(() -> driverController.getRawAxis(1) , () -> -driverController.getRawAxis(0) , () -> -driverController.getRawAxis(3))
            //swerve.driveJoystick(() -> -driverSim.getRawAxis(4) , () -> -driverSim.getRawAxis(3) , () -> -driverSim.getRawAxis(0))
            //swerve.driveJoystick(() -> -driverSim.getRawAxis(0) , () -> -driverSim.getRawAxis(1) , () -> -driverSim.getRawAxis(2))
        );

        //SmartDashboard.putNumber("SWR MTH: ", SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(3.161), 6.12, 1));

    }

    /**
     * Configure the bindings for the controller buttons to specific commands.
    */
    private void configureBindings(){

       // operatorController.y().onTrue( new CMD_IndexerManual(conveyor, true));
        //operatorController.y().onFalse(new CMD_IndexerStop(conveyor));

       // operatorController.x().onTrue( new CMD_IndexerManual(conveyor, false));
        //operatorController.x().onTrue(new CMD_IndexerStop(conveyor));

        
        /* 
        operatorController.x().onTrue(new CMDGR_Shoot(conveyor, arm, shooter, vision, led,
            ShooterMode.SPEAKER, () -> operatorController.b().getAsBoolean(), () -> operatorController.y().getAsBoolean()
        ));
        */

        operatorController.x().onTrue(new CMDGR_Shoot(conveyor, arm, shooter, vision, led,
            ShooterMode.SPEAKER, () -> operatorController.b().getAsBoolean(), () -> operatorController.y().getAsBoolean()
        ));
        

        operatorController.rightBumper().onTrue(new CMDGR_Shoot(conveyor, arm, shooter, vision, led,
            ShooterMode.AMP, () -> operatorController.b().getAsBoolean(), () -> operatorController.y().getAsBoolean()
        ));
        
        operatorController.leftBumper().onTrue(new CMD_IndexerManual(conveyor, false));

        //operatorController.a().onTrue(new CMD_Arm(arm, ArmState.INTAKE));

        operatorController.a().onTrue(new CMD_Intake(conveyor, arm, () -> operatorController.b().getAsBoolean()));

        //operatorController.a().debounce(0.25, DebounceType.kRising).onTrue(new CMD_Arm(arm, ArmState.INTAKE));

        //operatorController.a().debounce(0.25, DebounceType.kFalling).onFalse(new CMD_Arm(arm, ArmState.IDLE));

        operatorController.b().onTrue(new CMD_Idle(conveyor, arm, shooter));

        //operatorController.x().onTrue(new CMD_Intake(conveyor, arm, () -> operatorController.b().getAsBoolean()));

        
      // operatorController.b().onTrue(swerve.driveToPose(
      //  new Pose2d(2.30, 5.37, Rotation2d.fromDegrees(180))
       //));
    }

    /**
     * Get the autonomous command.
     * @return The autonomous command
    */
    public Command getAutonomousCommand(){
        return AutoBuilder.buildAuto("Test_Auto");
    }
}