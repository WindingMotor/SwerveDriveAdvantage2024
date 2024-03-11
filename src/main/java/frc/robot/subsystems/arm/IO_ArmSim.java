// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;

/** A super bare bone implementation of the arm just for simulation testing. */
public class IO_ArmSim implements IO_ArmBase {

	private Double armAngle;

	private final Mechanism2d armMechanism = new Mechanism2d(2, 2);
	private final MechanismRoot2d armPivot = armMechanism.getRoot("ArmPivot", 1, 1);

	private final MechanismLigament2d m_armTower =
			armPivot.append(new MechanismLigament2d("ArmTower", 1, -90));

	private MechanismLigament2d arm =
			armPivot.append(
					new MechanismLigament2d(
							"Arm", 1, Units.radiansToDegrees(2 * Math.PI), 6, new Color8Bit(Color.kYellow)));

	public IO_ArmSim() {

		armAngle = Constants.Maestro.ARM_OFFSET_DEGREES;

		SmartDashboard.putData("Arm Mechanism", armMechanism);
	}

	/**
	 * Updates the inputs with the current values.
	 *
	 * @param inputs The inputs to update.
	 */
	@Override
	public void updateInputs(ArmInputs inputs) {

		inputs.armPositionDegrees = armAngle;

		inputs.setpointPosition = armAngle;
		inputs.isAtSetpoint = true;

		inputs.motorOneCurrent = -1.0;
		inputs.motorTwoCurrent = -1.0;

		inputs.pidOutputVolts = -1.0;
		inputs.ffOutputVolts = -1.0;
		inputs.pidError = -1.0;

		inputs.isArmLocked = false;
		inputs.servoPosition = -1;
	}

	/**
	 * Updates the arm angle with the new setpoint position.
	 *
	 * @param newSetpointPosition The new position for the arm.
	 */
	@Override
	public void updatePID(double newSetpointPosition) {
		this.armAngle = newSetpointPosition;
		arm.setAngle(armAngle);
	}

	/** Stops the arm motors. */
	@Override
	public void stop() {
		updatePID(0.0);
	}

	/**
	 * Set the speed of the motors. Not implemented in simulation.
	 *
	 * @param speed the speed to set for the motors
	 */
	@Override
	public void setSpeed(double speed) {
		DriverStation.reportWarning("[sim] Setting the arm speed is not implemented", false);
	}

	/**
	 * Locks the arm shaft using a servo motor. Not implemented in simulation.
	 *
	 * @param enable Whether or not to lock the arm motors
	 */
	@Override
	public void lockArm() {
		DriverStation.reportWarning("[sim] Locking the arm is not implemented", false);
	}

	@Override
	public double getRealTimeArmPosition() {
		return armAngle;
	}
}
