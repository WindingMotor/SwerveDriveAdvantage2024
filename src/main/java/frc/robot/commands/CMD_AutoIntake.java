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

/** Command to control the intake process. */
public class CMD_AutoIntake extends Command {

	private final SUB_Conveyor conveyor;

	/**
	 * Constructs a new CMD_Intake command.
	 *
	 * @param conveyor The conveyor subsystem.
	 */
	public CMD_AutoIntake(SUB_Conveyor conveyor) {
		this.conveyor = conveyor;
		addRequirements(conveyor);
	}

	/**
	 * When command starts reset the isCommandDone flag, report to the driver station that the command
	 * is running, and set the robot subsystems to eject mode.
	 */
	@Override
	public void initialize() {}

	/**
	 * Every cycle check if the intake sensor is triggered. Once donut leaves the intake set the
	 * isCommandDone flag to true.
	 */
	@Override
	public void execute() {
		if (conveyor.inputs.intakeSensorState == true) {
			conveyor.setState(ConveyorState.INTAKE);
		}
	}

	/**
	 * Sets conveyor and arm states to IDLE when command ends.
	 *
	 * @param interrupted Whether the command was interrupted
	 */
	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() {
		return false;
	}
}
