// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SUB_Swerve;
import java.util.function.DoubleSupplier;

public class CMD_Rotate extends Command {

	private SUB_Swerve swerve;
	private PIDController pid;

	private DoubleSupplier xInput;
	private DoubleSupplier yInput;

	private double output;

	public CMD_Rotate(SUB_Swerve swerve, DoubleSupplier xInput, DoubleSupplier yInput) {
		this.swerve = swerve;
		this.xInput = xInput;
		this.yInput = yInput;

		addRequirements(swerve);

		this.pid = new PIDController(0.015, 0, 0.003);
		this.pid.enableContinuousInput(-180, 180);
		this.pid.setTolerance(0.5);

		this.output = 0.0;
	}

	// Print a message to the driver station and set the arm state
	@Override
	public void initialize() {
		output = 0.0;
	}

	@Override
	public void execute() {

		Rotation2d currentAngle = swerve.getPose().getRotation();

		output =
				pid.calculate(
						currentAngle.getDegrees(),
						Constants.Auto.ScoringPoses.BLU_AMP.pose.getRotation().getDegrees());

		// swerve.driveJoystick(xInput, yInput, () -> output);

		swerve.driveJoystickHybrid(xInput.getAsDouble(), yInput.getAsDouble(), output);
	}

	// Command ends immediately
	@Override
	public boolean isFinished() {
		return false;
	}
}
