// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.States.ArmState;
import frc.robot.subsystems.arm.SUB_Arm;

public class CMD_Arm extends Command {

  private SUB_Arm arm;
  private ArmState state;

  public CMD_Arm(SUB_Arm arm, ArmState state) {
    this.arm = arm;
    this.state = state;
    addRequirements(arm);
  }

  // Print a message to the driver station and set the arm state
  @Override
  public void initialize() {
    // Stop and idle the robot subsystems
    arm.setState(state);
  }

  // Command ends immediately
  @Override
  public boolean isFinished() {
    return true;
  }
}
