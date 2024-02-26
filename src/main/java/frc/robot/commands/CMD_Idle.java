// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.SUB_Arm;
import frc.robot.subsystems.conveyor.SUB_Conveyor;
import frc.robot.subsystems.shooter.SUB_Shooter;

public class CMD_Idle extends Command {

  private final SUB_Conveyor conveyor;
  private final SUB_Arm arm;
  private final SUB_Shooter shooter;

  public CMD_Idle(SUB_Conveyor conveyor, SUB_Arm arm, SUB_Shooter shooter) {
    this.conveyor = conveyor;
    this.arm = arm;
    this.shooter = shooter;
    addRequirements(conveyor, arm, shooter);
  }

  // Print a message to the driver station and idle the robot subsystems
  @Override
  public void initialize() {
    // Stop and idle the robot subsystems
    conveyor.setState(Constants.States.ConveyorState.OFF);
    arm.setState(Constants.States.ArmState.IDLE);
    shooter.setState(Constants.States.ShooterState.IDLE);
  }

  // Command ends immediately
  @Override
  public boolean isFinished() {
    return true;
  }
}
