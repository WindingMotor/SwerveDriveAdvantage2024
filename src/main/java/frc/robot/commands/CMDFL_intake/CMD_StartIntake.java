// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.CMDFL_intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.States.ConveyorState;
import frc.robot.subsystems.conveyor.SUB_Conveyor;

/** Command to control the intake process. */
public class CMD_StartIntake extends Command {

	private final SUB_Conveyor conveyor;

	/**
	 * Constructs a new CMD_Intake command.
	 *
	 * @param conveyor The conveyor subsystem.
	 * @param arm The arm subsystem.
	 * @param manualCancel The supplier to determine if the command should be manually cancelled.
	 */
	public CMD_StartIntake(SUB_Conveyor conveyor) {
		this.conveyor = conveyor;
		addRequirements(conveyor);
	}

	/**
	 * When command starts reset the isCommandDone flag, report to the driver station that the command
	 * is running, and set the robot subsystems to intake mode.
	 */
	@Override
	public void initialize() {
		conveyor.setState(ConveyorState.INTAKE);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
