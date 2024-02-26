// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.AddressableLedStrip;
import frc.robot.util.AddressableLedStrip.LEDState;

public class CMD_Led extends Command {

  private final AddressableLedStrip led;
  private final LEDState state;

  public CMD_Led(AddressableLedStrip led, LEDState state) {
    this.led = led;
    this.state = state;
    addRequirements(led);
  }

  // Print a message to the driver station and set the LED state
  @Override
  public void initialize() {
    led.setState(state);
  }

  // Command ends immediately
  @Override
  public boolean isFinished() {
    return true;
  }
}
