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

	private boolean DYNAMIC_MODE = false;
	private double dynamicAngle = 0;

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

		if (!CLIMB_MODE && !DYNAMIC_MODE) {
			io.updatePID(state.position);
		} else if (DYNAMIC_MODE) {
			io.updatePID(dynamicAngle);
		}
	}

	public void setState(ArmState newState) {
		DYNAMIC_MODE = false;
		state = newState;
	}

	/**
	 * Set the dynamic angle for the arm, dynamic mode will be enabled when this is set.
	 *
	 * @param newAngle the new angle to set
	 * @return void
	 */
	public void setDynamicAngle(double newAngle) {
		CLIMB_MODE = false;
		DYNAMIC_MODE = true;
		if (newAngle > 90) {
			newAngle = 90;
		}
		if (newAngle < -4) {
			newAngle = -4;
		}
		dynamicAngle = newAngle;
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

	/**
	 * Enables or disables the climb mode. When in climb mode, the arm will disable the PID
	 * controller.
	 *
	 * @param newClimbMode The new value for climb mode
	 */
	public void setClimbMode(boolean newClimbMode) {
		CLIMB_MODE = newClimbMode;
	}

	/**
	 * Sets the speed of arm, can only be used in CLIMB MODE.
	 *
	 * @param speed The speed to set
	 */
	public void setSpeed(double speed) {
		io.setSpeed(speed);
	}

	/** Locks the arm using a the servo motor. THIS ACTION IS IRREVERSIBLE */
	public void lockArm() {
		io.lockArm();
	}

	/**
	 * Retrieves the real-time position of the arm.
	 *
	 * @return The real-time position of the arm in degrees
	 */
	public double getRealTimeArmPosition() {
		return io.getRealTimeArmPosition();
	}
}
