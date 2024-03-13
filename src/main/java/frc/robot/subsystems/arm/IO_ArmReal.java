// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.Builder;
import org.littletonrobotics.junction.Logger;

public class IO_ArmReal implements IO_ArmBase {

	private CANSparkMax motorOne;
	private CANSparkMax motorTwo;

	private ProfiledPIDController pid;

	private ArmFeedforward feedforward;

	private double setpointPosition;

	private Encoder armEncoder;

	private Double armAngle;

	private double pidOutputVolts;
	private double ffOutputVolts;

	private Servo servo;

	public IO_ArmReal() {

		pidOutputVolts = 0;
		ffOutputVolts = 0;

		motorOne =
				Builder.createNeo(
						Constants.Maestro.ARM_MOTOR_LEAD_ID, Constants.Maestro.ARM_MOTOR_LEAD_INVERTED, 40);
		motorTwo =
				Builder.setNeoFollower(
						motorOne,
						Builder.createNeo(
								Constants.Maestro.ARM_MOTOR_FOLLOWER_ID,
								Constants.Maestro.ARM_MOTOR_FOLLOWER_INVERTED,
								40));

		Builder.configureIdleMode(motorOne, false);
		Builder.configureIdleMode(motorTwo, false);

		pid =
				new ProfiledPIDController(
						Constants.Maestro.ARM_P,
						Constants.Maestro.ARM_I,
						Constants.Maestro.ARM_D,
						new Constraints(
								Constants.Maestro.ARM_MAX_VELOCITY, Constants.Maestro.ARM_MAX_ACCELERATION));

		feedforward =
				new ArmFeedforward(
						Constants.Maestro.ARM_KS,
						Constants.Maestro.ARM_KG,
						Constants.Maestro.ARM_KV,
						Constants.Maestro.ARM_KA);

		armEncoder = new Encoder(7, 8, true, Encoder.EncodingType.k2X);
		armEncoder.setDistancePerPulse(0.1);
		armEncoder.reset();
		armAngle = armEncoder.getDistance() - Constants.Maestro.ARM_OFFSET_DEGREES;

		if (Constants.PID_TEST_MODE) {
			SmartDashboard.putNumber("armTestAngleInput", 0);
		}

		Builder.configureIdleMode(motorOne, true);
		Builder.configureIdleMode(motorTwo, true);

		servo = new Servo(Constants.Maestro.ARM_SERVO_PORT);
	}

	/**
	 * Updates the inputs with the current vsalues.
	 *
	 * @param inputs The inputs to update.
	 */
	@Override
	public void updateInputs(ArmInputs inputs) {

		armAngle = armEncoder.getDistance() - Constants.Maestro.ARM_OFFSET_DEGREES;
		inputs.armPositionDegrees = armAngle;

		inputs.setpointPosition = setpointPosition;
		inputs.isAtSetpoint =
				Math.abs(armAngle - setpointPosition) < Constants.Maestro.ARM_TOLERANCE_DEGREES;

		inputs.motorOneCurrent = motorOne.getOutputCurrent();
		inputs.motorTwoCurrent = motorTwo.getOutputCurrent();

		inputs.pidOutputVolts = pidOutputVolts;
		inputs.ffOutputVolts = ffOutputVolts;
		inputs.pidError = pid.getPositionError();

		inputs.isArmLocked = servo.getPosition() > 0.1;
		inputs.servoPosition = servo.getPosition();
	}

	/**
	 * Updates the PID controller with the new setpoint position.
	 *
	 * @param newSetpointPosition The new setpoint position for the PID controller
	 */
	@Override
	public void updatePID(double newSetpointPosition) {

		this.setpointPosition = newSetpointPosition;

		// Live debug code
		if (Constants.PID_TEST_MODE) {
			double armTestAngleInput = SmartDashboard.getNumber("armTestAngleInput", 0);
			if (armTestAngleInput > 110) {
				armTestAngleInput = 110;
				SmartDashboard.putNumber("armTestAngleInput", 110);
			}

			Logger.recordOutput("Test Setpoint Degrees", armTestAngleInput);

			runPID(armTestAngleInput);
		} else {
			runPID(setpointPosition);
		}

		motorOne.setVoltage(pidOutputVolts + ffOutputVolts);
	}

	private void runPID(double setpointDegrees) {
		pidOutputVolts =
				MathUtil.clamp(
						-pid.calculate(armAngle, setpointDegrees),
						-Constants.Maestro.ARM_VOLTAGE_CLAMPING,
						Constants.Maestro.ARM_VOLTAGE_CLAMPING);
		ffOutputVolts = -feedforward.calculate(Math.toRadians(setpointDegrees), 0.0);
	}

	/** Stops the arm motors. */
	@Override
	public void stop() {
		updatePID(0.0);
	}

	/**
	 * Set the speed of the motors.
	 *
	 * @param speed the speed to set for the motors
	 */
	@Override
	public void setSpeed(double speed) {
		motorOne.set(speed);
		motorTwo.set(speed);
	}

	/**
	 * Locks the arm shaft using a servo motor. THIS ACTION IS IRREVERSIBLE.
	 *
	 * @param enable Whether or not to lock the arm motors
	 */
	@Override
	public void lockArm() {
		servo.setPosition(50);
	}

	@Override
	public void unlockArm() {
		servo.setPosition(0);
	}

	@Override
	public double getRealTimeArmPosition() {
		return armAngle;
	}
}
