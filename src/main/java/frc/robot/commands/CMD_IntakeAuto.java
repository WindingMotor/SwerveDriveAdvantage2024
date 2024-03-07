// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.SUB_Arm;
import frc.robot.subsystems.conveyor.SUB_Conveyor;
import java.util.function.Supplier;

/** Command to control the intake process. */
public class CMD_IntakeAuto extends Command {

	private final SUB_Conveyor conveyor;
	private final SUB_Arm arm;
	private final Supplier<Boolean> manualCancel;
	private boolean isCommandDone = false;
	private Debouncer debouncer;

	/**
	 * Constructs a new CMD_Intake command.
	 *
	 * @param conveyor The conveyor subsystem.
	 * @param arm The arm subsystem.
	 * @param manualCancel The supplier to determine if the command should be manually cancelled.
	 */
	public CMD_IntakeAuto(SUB_Conveyor conveyor, SUB_Arm arm, Supplier<Boolean> manualCancel) {
		this.conveyor = conveyor;
		this.arm = arm;
		this.manualCancel = manualCancel;
		debouncer = new Debouncer(0.025, Debouncer.DebounceType.kRising);

		addRequirements(conveyor, arm);
	}

	/**
	 * When command starts reset the isCommandDone flag, report to the driver station that the command
	 * is running, and set the robot subsystems to intake mode.
	 */
	@Override
	public void initialize() {
		arm.setClimbMode(false);
		isCommandDone = false;
		conveyor.setState(Constants.States.ConveyorState.INTAKE);
		arm.setState(Constants.States.ArmState.INTAKE);
	}

	/**
	 * Every cycle check if the indexer sensor is triggered. Once donut enters the indexer set the
	 * isCommandDone flag to true.
	 */
	@Override
	public void execute() {
		// If the indexer sensor is triggered, end the command
		if (debouncer.calculate(conveyor.inputs.indexerInitalSensorState)) {
			isCommandDone = true;
		}
	}

	/**
	 * Sets conveyor and arm states to IDLE when command ends.
	 *
	 * @param interrupted Whether the command was interrupted
	 */
	@Override
	public void end(boolean interrupted) {
		conveyor.setState(Constants.States.ConveyorState.OFF);
	}

	@Override
	public boolean isFinished() {
		if (isCommandDone || manualCancel.get()) {
			return true;
		} else {
			return false;
		}
	}
}
