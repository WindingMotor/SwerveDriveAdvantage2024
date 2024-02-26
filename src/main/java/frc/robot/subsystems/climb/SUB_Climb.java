// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class SUB_Climb extends SubsystemBase {

  private final IO_ClimbBase io;

  public final ClimbInputsAutoLogged inputs = new ClimbInputsAutoLogged();

  public SUB_Climb(IO_ClimbBase io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);
  }

  public void stop() {
    io.stop();
  }

  public void set(double speed) {
    io.set(speed);
  }
}
