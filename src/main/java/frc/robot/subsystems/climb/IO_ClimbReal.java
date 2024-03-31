// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climb;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;
import frc.robot.util.Builder;

/** Represents a real implementation of the shooter. */
public class IO_ClimbReal implements IO_ClimbBase {

	private CANSparkMax motor;

	private RelativeEncoder motorEncoder;

	public IO_ClimbReal() {

		motor =
				Builder.createNeo(
						Constants.Maestro.CLIMB_MOTOR_ID, Constants.Maestro.CLIMB_MOTOR_INVERTED, 40);

		Builder.configureIdleMode(motor, true);

		motorEncoder = motor.getEncoder();

		motorEncoder.setPosition(0.0);
	}

	/**
	 * Updates the inputs with the current values.
	 *
	 * @param inputs The inputs to update
	 */
	@Override
	public void updateInputs(ClimbInputs inputs) {
		inputs.motorPosition = motorEncoder.getPosition();
		inputs.isAtClimbPosition = false;
	}

	/** Stops the shooter by setting the PID setpoint to 0. */
	@Override
	public void stop() {
		motor.set(0);
	}

	/** Sets the setpoint RPM. */
	@Override
	public void set(double speed) {
		motor.set(speed);
	}
}
