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
import java.util.function.Supplier;

/** Command to control the intake process. */
public class CMD_ArmClimbRaise extends Command {

	private final SUB_Arm arm;

	/**
	 * Constructs a new CMD_Intake command.
	 *
	 * @param arm The arm subsystem.
	 * @param manualCancel The supplier to determine if the command should be manually cancelled.
	 */
	public CMD_ArmClimbRaise(SUB_Arm arm, Supplier<Boolean> manualCancel) {
		this.arm = arm;

		addRequirements(arm);
	}

	/**
	 * When command starts reset the isCommandDone flag, report to the driver station that the command
	 * is running, and set the robot subsystems to intake mode.
	 */
	@Override
	public void initialize() {
		arm.setClimbMode(false);
		arm.setState(Constants.States.ArmState.INTAKE);
	}

	/**
	 * Every cycle check if the indexer sensor is triggered. Once donut enters the indexer set the
	 * isCommandDone flag to true.
	 */
	@Override
	public void execute() {}

	/**
	 * Sets conveyor and arm states to IDLE when command ends.
	 *
	 * @param interrupted Whether the command was interrupted
	 */
	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() {
		return true;
	}
}
