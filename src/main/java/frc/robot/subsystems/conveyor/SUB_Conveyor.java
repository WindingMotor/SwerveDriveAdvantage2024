// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.conveyor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.States.ConveyorState;
import org.littletonrobotics.junction.Logger;

public class SUB_Conveyor extends SubsystemBase {

	private final IO_ConveyorBase io;

	public final ConveyorInputsAutoLogged inputs = new ConveyorInputsAutoLogged();

	private ConveyorState state;

	public SUB_Conveyor(IO_ConveyorBase io) {
		this.io = io;
		state = ConveyorState.OFF;
	}

	@Override
	public void periodic() {
		// Update the inputs.
		io.updateInputs(inputs);

		// Process inputs and send to logger.
		Logger.processInputs("Conveyor", inputs);

		io.update(state.intakeSpeed, state.indexerSpeed);
	}

	public void setState(ConveyorState newState) {
		state = newState;
	}

	public ConveyorState getState() {
		return state;
	}

	public boolean getShooterFlag() {
		return inputs.shooterFlag;
	}

	public void stop() {
		io.stop();
	}
}
