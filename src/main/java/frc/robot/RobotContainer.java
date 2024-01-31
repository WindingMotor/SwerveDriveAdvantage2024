
 package frc.robot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.States.ShooterMode;
import frc.robot.beluga.arm.IO_ArmReal;
import frc.robot.beluga.arm.SUB_Arm;
import frc.robot.beluga.conveyor.CMD_IndexerManual;
import frc.robot.beluga.conveyor.CMD_IndexerStop;
import frc.robot.beluga.conveyor.IO_ConveyorReal;
import frc.robot.beluga.conveyor.SUB_Conveyor;
import frc.robot.beluga.shooter.IO_ShooterReal;
import frc.robot.beluga.shooter.SUB_Shooter;
import frc.robot.command.CMDGR_Shoot;
import frc.robot.command.CMD_Idle;
import frc.robot.wmlib2j.util.AddressableLedStrip;
import frc.robot.wmlib2j.vision.IO_VisionReal;
import frc.robot.wmlib2j.vision.SUB_Vision;
import frc.robot.yagsl.IO_SwerveReal;
import frc.robot.yagsl.SUB_Swerve;

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

    private final SUB_Conveyor intexer = new SUB_Conveyor(
        new IO_ConveyorReal()
    );

    private final AddressableLedStrip led = new AddressableLedStrip(0, 15);

    public RobotContainer(){

        configureBindings();
        
        swerve.setDefaultCommand( 
            swerve.driveJoystick(() -> -driverController.getRawAxis(1) , () -> driverController.getRawAxis(0) , () -> -driverController.getRawAxis(3))
            //swerve.driveJoystick(() -> -driverSim.getRawAxis(4) , () -> -driverSim.getRawAxis(3) , () -> -driverSim.getRawAxis(0))
            //swerve.driveJoystick(() -> -driverSim.getRawAxis(0) , () -> -driverSim.getRawAxis(1) , () -> -driverSim.getRawAxis(2))
        );
    }

    /**
     * Configure the bindings for the controller buttons to specific commands.
    */
    private void configureBindings(){

        operatorController.y().onTrue( new CMD_IndexerManual(intexer, true));
        operatorController.y().onFalse(new CMD_IndexerStop(intexer));

        operatorController.x().onTrue( new CMD_IndexerManual(intexer, false));
        operatorController.x().onTrue(new CMD_IndexerStop(intexer));

        operatorController.a().onTrue(new CMDGR_Shoot(intexer, arm, shooter, vision, led,
            ShooterMode.SPEAKER, () -> operatorController.a().getAsBoolean()
        ));

        operatorController.b().onTrue(new CMD_Idle(intexer, arm, shooter));
    }

    /**
     * Get the autonomous command.
     * @return The autonomous command
    */
    public Command getAutonomousCommand(){
        return swerve.getAutonomousCommand("Test", true);
    }
}