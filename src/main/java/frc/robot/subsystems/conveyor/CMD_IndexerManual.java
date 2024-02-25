// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.conveyor;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

/* Manual control of the indexer. */
public class CMD_IndexerManual extends Command {

  private final SUB_Conveyor conveyor;
  private final boolean isReversed;

  public CMD_IndexerManual(SUB_Conveyor conveyor, boolean isReversed) {
    this.conveyor = conveyor;
    this.isReversed = isReversed;
    addRequirements(conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (isReversed) {
      conveyor.setState(Constants.States.ConveyorState.EJECT);
    } else {
      conveyor.setState(Constants.States.ConveyorState.INTAKE);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    conveyor.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
