
package frc.robot.command;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.States.ShooterMode;
import frc.robot.Constants.States.ShooterState;
import frc.robot.beluga.arm.SUB_Arm;
import frc.robot.beluga.conveyor.SUB_Conveyor;
import frc.robot.beluga.shooter.SUB_Shooter;
import frc.robot.wmlib2j.util.AddressableLedStrip;
import frc.robot.wmlib2j.util.AddressableLedStrip.LEDState;
import frc.robot.wmlib2j.vision.SUB_Vision;

public class CMDGR_Shoot extends SequentialCommandGroup{
  
  public CMDGR_Shoot(SUB_Conveyor intexer, SUB_Arm arm, SUB_Shooter shooter, SUB_Vision vision, AddressableLedStrip led, ShooterMode mode, Supplier<Boolean> manualCancel){
    addRequirements(intexer, arm, shooter);
      // Call the shoot command, then wait 0.5 seconds, then call the idle command
      addCommands(
        new CMD_Led(led, LEDState.GREEN),
        new CMD_Shoot(intexer, arm, shooter, vision, mode, manualCancel),
        new WaitCommand(0.5), // Delay to allow dount to leave the robot
        new CMD_Idle(intexer, arm, shooter),
        new CMD_Led(led, LEDState.ALLIANCE)

      
      );
  }
}
