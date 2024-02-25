// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.States.ArmState;
import org.littletonrobotics.junction.Logger;

public class SUB_Arm extends SubsystemBase {

  private final IO_ArmBase io;

  public final ArmInputsAutoLogged inputs = new ArmInputsAutoLogged();

  private ArmState state;

  public SUB_Arm(IO_ArmBase io) {
    this.io = io;
    state = ArmState.OFF;
    ;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
    io.updatePID(state.position);
  }

  public void setState(ArmState newState) {
    state = newState;
  }

  public ArmState getState() {
    return state;
  }

  public void stop() {
    io.stop();
  }

  public boolean isAtSetpoint() {
    return inputs.isAtSetpoint;
  }
}
