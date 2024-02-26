// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/**
 * Base interface for the shooter, abstracts to the real and simulation classes. Provides methods
 * for updating inputs.
 */
public interface IO_ShooterBase {

	/**
	 * Represents the inputs for a gyroscope. It contains fields for yaw, pitch, and roll positions.
	 */
	@AutoLog
	public static class ShooterInputs {
		// RPM = Rotation/Minute
		public double motorOneRPM = 0.0;
		public double motorTwoRPM = 0.0;
		public double setpointRPM = 0.0;
		public boolean isUpToSpeed = false;
	}

	/**
	 * Updates the inputs with the current values.
	 *
	 * @param inputs The inputs to update.
	 */
	void updateInputs(ShooterInputs inputs);

	void updatePID(double setpointRPM);

	void stop();

	void setRPM(double rpm);

	boolean isUpToSpeed();

	void invertMotors(boolean inverted);
}
