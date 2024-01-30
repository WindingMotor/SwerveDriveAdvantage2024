
package frc.robot.command;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.States.ShooterMode;
import frc.robot.beluga.arm.SUB_Arm;
import frc.robot.beluga.conveyor.SUB_Conveyor;
import frc.robot.beluga.shooter.SUB_Shooter;

public class CMDGR_Shoot extends SequentialCommandGroup{


  public CMDGR_Shoot(SUB_Conveyor intexer, SUB_Arm arm, SUB_Shooter shooter, ShooterMode mode, Supplier<Boolean> manualCancel){
    addRequirements(intexer, arm, shooter);
    // Call the shoot command, then wait 0.5 seconds, then call the idle command
    addCommands(
      new CMD_Shoot(intexer, arm, shooter, mode, manualCancel),
      new WaitCommand(0.5), // Delay to allow dount to leave the robot
      new CMD_Idle(intexer, arm, shooter)
    );

  }

  // Called when the command is initially scheduled.
}
