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

	private CANSparkMax leftMotor;
	private CANSparkMax rightMotor;

	private RelativeEncoder leftMotorEncoder;
	private RelativeEncoder rightMotorEncoder;

	private double setpointRPM;

	public IO_ClimbReal() {

		setpointRPM = 0;

		leftMotor =
				Builder.createNeo(
						Constants.Maestro.LEFT_CLIMB_MOTOR_ID, Constants.Maestro.LEFT_CLIMB_MOTOR_INVERTED, 40);
		rightMotor =
				Builder.createNeo(
						Constants.Maestro.RIGHT_CLIMB_MOTOR_ID,
						Constants.Maestro.RIGHT_CLIMB_MOTOR_INVERTED,
						40);

		Builder.configureIdleMode(leftMotor, true);
		Builder.configureIdleMode(rightMotor, true);

		leftMotorEncoder = leftMotor.getEncoder();
		rightMotorEncoder = rightMotor.getEncoder();
	}

	/**
	 * Updates the inputs with the current values.
	 *
	 * @param inputs The inputs to update
	 */
	@Override
	public void updateInputs(ClimbInputs inputs) {

		inputs.leftMotorPosition = leftMotorEncoder.getVelocity();
		inputs.rightMotorPosition = rightMotorEncoder.getVelocity();
		inputs.isAtClimbPosition = false;
	}

	/** Stops the shooter by setting the PID setpoint to 0. */
	@Override
	public void stop() {
		leftMotor.set(0);
		rightMotor.set(0);
	}

	/** Sets the setpoint RPM. */
	@Override
	public void set(double speed) {
		leftMotor.set(speed);
		rightMotor.set(speed);
	}
}
