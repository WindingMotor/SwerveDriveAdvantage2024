// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.States.ShooterState;
import org.littletonrobotics.junction.Logger;

public class SUB_Shooter extends SubsystemBase {

	private final IO_ShooterBase io;

	public final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

	private ShooterState state;

	private boolean DYNAMIC_MODE = false;
	private double dynamicRPM = 0;

	public SUB_Shooter(IO_ShooterBase io) {
		this.io = io;
		this.state = ShooterState.OFF;
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Shooter", inputs);

		if (!DYNAMIC_MODE) {
			io.updatePID(state.rpm);
		} else {
			io.updatePID(dynamicRPM);
		}
	}

	public void setState(ShooterState newState) {
		DYNAMIC_MODE = false;
		state = newState;
	}

	public void setDynamicRPM(double rpm) {
		DYNAMIC_MODE = true;
		dynamicRPM = rpm;
	}

	public ShooterState getState() {
		return state;
	}

	public void stop() {
		io.stop();
	}

	public void invertMotors(boolean inverted) {
		io.invertMotors(inverted);
	}

	public boolean isUptoSpeed() {
		return io.isUpToSpeed();
	}
}
