
package frc.robot.command;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.wmlib2j.util.AddressableLedStrip;
import frc.robot.wmlib2j.util.AddressableLedStrip.LEDState;

public class CMD_Led extends Command{

  private final AddressableLedStrip led;
  private final LEDState state;

  public CMD_Led(AddressableLedStrip led, LEDState state){
    this.led = led;
    this.state = state;
    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    DriverStation.reportWarning("[init] CMD_Led switching to " + state.toString() + " state", false);
    led.setState(state);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){}

  // Returns true when the command should end.
  @Override
  public boolean isFinished(){ return true;}

}