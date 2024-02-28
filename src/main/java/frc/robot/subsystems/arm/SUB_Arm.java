// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

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

	private boolean CLIMB_MODE = false;

	public SUB_Arm(IO_ArmBase io) {
		this.io = io;
		state = ArmState.OFF;
		;
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Arm", inputs);

		if (!CLIMB_MODE) {
			io.updatePID(state.position);
		}
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

	public void setClimbMode(boolean newClimbMode) {
		CLIMB_MODE = newClimbMode;
	}

	public void setSpeed(double speed) {
		io.setSpeed(speed);
	}
}
