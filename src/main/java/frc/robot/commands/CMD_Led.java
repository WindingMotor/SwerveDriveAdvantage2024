// Written by WindingMotor as part of the wmlib2j library.

package frc.robot.commands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.AddressableLedStrip;
import frc.robot.util.AddressableLedStrip.LEDState;

public class CMD_Led extends Command{

  private final AddressableLedStrip led;
  private final LEDState state;

  public CMD_Led(AddressableLedStrip led, LEDState state){
    this.led = led;
    this.state = state;
    addRequirements(led);
  }

  // Print a message to the driver station and set the LED state
  @Override
  public void initialize(){
    DriverStation.reportWarning("[init] CMD_Led switching to " + state.toString() + " state", false);
    led.setState(state);
  }

  // Command ends immediately
  @Override
  public boolean isFinished(){ return true; }

}