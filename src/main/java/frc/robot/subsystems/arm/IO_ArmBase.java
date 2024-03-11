// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface IO_ArmBase {

	@AutoLog
	public static class ArmInputs {

		public double armPositionDegrees = 0.0;

		public double motorOneCurrent = 0.0;
		public double motorTwoCurrent = 0.0;

		public double setpointPosition = 0.0;
		public boolean isAtSetpoint = false;

		public double pidOutputVolts = 0.0;
		public double ffOutputVolts = 0.0;
		public double pidError = 0.0;

		public boolean isArmLocked = false;
		public double servoPosition = 0.0;
	}

	/**
	 * Updates the inputs with the current values.
	 *
	 * @param inputs The inputs to update.
	 */
	void updateInputs(ArmInputs inputs);

	void updatePID(double newSetpoint);

	void stop();

	void setSpeed(double speed);

	void lockArm();

	double getRealTimeArmPosition();
}
