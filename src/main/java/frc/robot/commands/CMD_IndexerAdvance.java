// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.States.ConveyorState;
import frc.robot.subsystems.conveyor.SUB_Conveyor;

/** Class to handle shooting commands. */
public class CMD_IndexerAdvance extends Command {

	private final SUB_Conveyor conveyor;
	private boolean isCommandDone = false;

	public CMD_IndexerAdvance(SUB_Conveyor conveyor) {
		this.conveyor = conveyor;
		addRequirements(conveyor);
	}

	@Override
	public void initialize() {
		conveyor.setState(ConveyorState.SLOW);
		isCommandDone = false;
	}

	@Override
	public void execute() {
		if (conveyor.inputs.indexerFinalSensorState) {
			conveyor.setState(ConveyorState.OFF);
			isCommandDone = true;
		}
	}

	@Override
	public boolean isFinished() {
		return isCommandDone;
	}
}
