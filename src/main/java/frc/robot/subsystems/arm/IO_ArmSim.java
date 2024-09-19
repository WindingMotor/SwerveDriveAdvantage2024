// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class IO_ArmSim implements IO_ArmBase {

	private final Mechanism2d mech2d;
	private final MechanismLigament2d armLigament;

	private static final double ARM_LENGTH = 0.5; // meters

	private double appliedVolts = 0.0;
	private double currentAngle = 0.0;
	private double setpointAngle = 0.0;
	private double velocity = 0.0;
	private boolean isLocked = false;
	private double servoPosition = 0.0;

	private final PIDController pidController;

	public IO_ArmSim() {
		mech2d = new Mechanism2d(3, 3);
		MechanismRoot2d root = mech2d.getRoot("ArmRoot", 1.5, 0.5);
		armLigament =
				root.append(new MechanismLigament2d("Arm", ARM_LENGTH, 90, 6, new Color8Bit(255, 255, 0)));

		SmartDashboard.putData("Arm Mechanism", mech2d);

		pidController =
				new PIDController(
						Constants.Maestro.ARM_P, Constants.Maestro.ARM_I, Constants.Maestro.ARM_D);
	}

	@Override
	public void updateInputs(ArmInputs inputs) {
		currentAngle += velocity * 0.02; // Simple simulation of arm movement
		currentAngle = Math.min(Math.max(currentAngle, -8.0), 200.0); // Clamp to min/max angles

		inputs.armPositionDegrees = currentAngle;
		inputs.motorOneCurrent = Math.abs(appliedVolts) * 0.5; // Simulated current
		inputs.motorTwoCurrent = Math.abs(appliedVolts) * 0.5; // Simulated current
		inputs.setpointPosition = setpointAngle;
		inputs.isAtSetpoint =
				Math.abs(currentAngle - setpointAngle) < Constants.Maestro.ARM_TOLERANCE_DEGREES;
		inputs.pidOutputVolts = pidController.calculate(currentAngle, setpointAngle);
		inputs.ffOutputVolts = 0; // Simplified simulation without feedforward
		inputs.pidError = setpointAngle - currentAngle;
		inputs.isArmLocked = isLocked;
		inputs.servoPosition = servoPosition;

		armLigament.setAngle(currentAngle);
		Logger.recordOutput("ArmMechanism", mech2d);
	}

	@Override
	public void updatePID(double newSetpoint) {
		setpointAngle = newSetpoint;
		double pidOutput = pidController.calculate(currentAngle, setpointAngle);
		setArmVoltage(pidOutput);
	}

	@Override
	public void stop() {
		setArmVoltage(0);
	}

	@Override
	public void setSpeed(double speed) {
		setArmVoltage(speed * 12); // Assuming 12V max
	}

	@Override
	public void lockArm() {
		isLocked = true;
		servoPosition = 50;
	}

	@Override
	public void unlockArm() {
		isLocked = false;
		servoPosition = 0;
	}

	@Override
	public double getRealTimeArmPosition() {
		return currentAngle;
	}

	@Override
	public double getRealTimeArmSetpoint() {
		return setpointAngle;
	}

	@Override
	public void setCurrentLimits(int limit) {
		// Not applicable in simulation
	}

	private void setArmVoltage(double volts) {
		appliedVolts = volts;
		// Simplified simulation: update velocity based on applied voltage
		velocity = volts * 10; // Adjust this factor to control sensitivity
	}
}
