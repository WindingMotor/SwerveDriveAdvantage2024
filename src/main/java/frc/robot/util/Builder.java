// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

/**
 * The Builder class provides static methods to create different types of hardware. Example usage:
 * CANSparkMax neoMotor = Builder.createNeo(1, false, 40); RelativeEncoder encoder =
 * Builder.createEncoder(neoMotor, 0.5, 1.0); SparkPIDController pidController = new
 * SparkPIDController(); Builder.configurePIDController(pidController, false);
 */
public class Builder {

  /**
   * Creates a motor with the given CAN ID and inversion setting.
   *
   * @param id The CAN ID of the motor.
   * @param inverted Whether the direction motor is inverted.
   * @param currentLimit The current limit for the motor.
   * @return The created Neo motor.
   */
  public static CANSparkMax createNeo(int id, boolean inverted, int currentLimit) {
    CANSparkMax motor = new CANSparkMax(id, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kCoast);
    motor.setSmartCurrentLimit(currentLimit);
    motor.setInverted(inverted);
    return motor;
  }

  /**
   * Creates a Vortex motor with the given parameters.
   *
   * @param id The unique ID of the motor
   * @param inverted true if the motor should be inverted, false otherwise
   * @param currentLimit The current limit for the motor
   * @return The created Vortex motor
   */
  public static CANSparkFlex createVortex(int id, boolean inverted, int currentLimit) {
    CANSparkFlex motor = new CANSparkFlex(id, MotorType.kBrushless);
    motor.restoreFactoryDefaults();
    motor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);
    motor.setSmartCurrentLimit(currentLimit);
    motor.setInverted(inverted);
    return motor;
  }

  /**
   * Creates an encoder with the given motor and conversion factors.
   *
   * @param motor The motor to bind the encoder to.
   * @param positionConversionFactor The conversion factor for the position.
   * @param velocityConversionFactor The conversion factor for the velocity.
   * @return The created encoder.
   */
  public static RelativeEncoder createEncoder(
      CANSparkBase motor, double positionConversionFactor, double velocityConversionFactor) {
    RelativeEncoder encoder = motor.getEncoder();
    encoder.setPositionConversionFactor(positionConversionFactor);
    encoder.setVelocityConversionFactor(velocityConversionFactor);
    return encoder;
  }

  /**
   * Configures a Spark PID controller.
   *
   * @param pid The SparkPIDController to be configured
   * @param wrap Wrap encoder output from 0 to 2PI
   * @param p The proportional gain
   * @param i The integral gain
   * @param d The derivative gain
   */
  public static SparkPIDController configurePIDController(
      SparkPIDController pid, boolean wrap, double p, double i, double d) {
    pid.setP(p);
    pid.setI(i);
    pid.setD(d);
    pid.setOutputRange(-1.0, 1.0);
    if (wrap) {
      pid.setOutputRange(0.0, 2 * Math.PI);
    }
    return pid;
  }

  /**
   * Configures a Spark PID controller.
   *
   * @param pid The SparkPIDController to be configured
   * @param wrap Wrap encoder output from 0 to 2PI
   * @param pidConstants The PID constants
   */
  public static SparkPIDController configurePIDController(
      SparkPIDController pid, boolean wrap, PIDConstants pidConstants) {
    pid.setP(pidConstants.kP);
    pid.setI(pidConstants.kI);
    pid.setD(pidConstants.kD);
    pid.setOutputRange(-1.0, 1.0);
    if (wrap) {
      pid.setOutputRange(0.0, 2 * Math.PI);
    }
    return pid;
  }

  /**
   * Configures a Spark PID controller.
   *
   * @param pid The SparkPIDController to be configured
   * @param wrap Wrap encoder output from 0 to 2PI
   * @param pidConstants The PID constants
   */
  public static SparkPIDController configurePIDController(
      SparkPIDController pid, boolean wrap, PIDConstants pidConstants, double iz, double ff) {
    pid.setP(pidConstants.kP);
    pid.setI(pidConstants.kI);
    pid.setD(pidConstants.kD);
    pid.setIZone(iz);
    pid.setFF(ff);
    pid.setOutputRange(-1.0, 1.0);
    if (wrap) {
      pid.setOutputRange(0.0, 2 * Math.PI);
    }
    return pid;
  }

  /**
   * Sets the provided Spark Max as a follower of the specified master Spark Max.
   *
   * @param master The master Spark Max
   * @param follower The Spark Max to set as a follower
   * @return The modified follower Spark Max
   */
  public static CANSparkMax setNeoFollower(CANSparkMax master, CANSparkMax follower) {
    follower.follow(master);
    return follower;
  }

  /**
   * Configures the idle mode of the motor, either brake or coast.
   *
   * @param motor The motor to configure
   * @param isBrake True if the idle mode should be set to brake, false if it should be set to coast
   */
  public static void configureIdleMode(CANSparkBase motor, boolean isBrake) {
    if (isBrake) {
      motor.setIdleMode(IdleMode.kBrake);
    } else {
      motor.setIdleMode(IdleMode.kCoast);
    }
  }
}
