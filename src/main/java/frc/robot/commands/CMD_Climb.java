// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.SUB_Climb;
import frc.robot.util.AddressableLedStrip;
import frc.robot.util.AddressableLedStrip.LEDState;

public class CMD_Climb extends Command {

  private final AddressableLedStrip led;
  private final SUB_Climb climb;
  Supplier<Double> speed;

  public CMD_Climb(AddressableLedStrip led, SUB_Climb climb, Supplier<Double> speed) {
    this.led = led;
    this.climb = climb;
    this.speed = speed;
    addRequirements(led);
  }

  // Print a message to the driver station and set the LED state
  @Override
  public void initialize() {
    if(speed.get() > 0.05){
      climb.set(speed.get());
      led.setState(LEDState.BLUE);
    }else{
      climb.stop();
      led.setState(LEDState.RAINBOW);
    }
  }

  // Command ends immediately
  @Override
  public boolean isFinished() {
    return true;
  }
}
