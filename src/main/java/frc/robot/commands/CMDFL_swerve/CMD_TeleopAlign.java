// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.CMDFL_swerve;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SUB_Swerve;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class CMD_TeleopAlign extends Command {

	private SUB_Swerve swerve;
	private PIDController pid;

	private DoubleSupplier xInput;
	private DoubleSupplier yInput;
	private Supplier<Boolean> manualCancel;
	private boolean isCommandDone = false;

	public CMD_TeleopAlign(
			SUB_Swerve swerve,
			DoubleSupplier xInput,
			DoubleSupplier yInput,
			Supplier<Boolean> manualCancel) {
		this.swerve = swerve;
		this.xInput = xInput;
		this.yInput = yInput;
		this.manualCancel = manualCancel;

		addRequirements(swerve);

		this.pid =
				new PIDController(
						Constants.Auto.SWERVE_ALIGN_PID.kP,
						Constants.Auto.SWERVE_ALIGN_PID.kI,
						Constants.Auto.SWERVE_ALIGN_PID.kD);

		this.pid.enableContinuousInput(-180, 180);
		this.pid.setTolerance(0.5);
	}

	// Print a message to the driver station and set the arm state
	@Override
	public void initialize() {
		isCommandDone = false;
		pid.reset();
	}

	@Override
	public void execute() {

		double currentAngleDegrees = swerve.getPose().getRotation().getDegrees();

		Pair<Rotation2d, Double> calculation = swerve.calculateAngleToSpeaker();

		double setpointDegrees = calculation.getFirst().getDegrees();
		double optimalEndingAngleDegrees = calculation.getSecond();

		double output = pid.calculate(currentAngleDegrees, setpointDegrees);

		swerve.driveRaw(0.0, 0.0, output);

		Logger.recordOutput("[CMD_TeleopAlign] Calculation Setpoint Degrees", setpointDegrees);
		Logger.recordOutput(
				"[CMD_TeleopAlign] Optimal Ending Angle Degrees  Setpoint", optimalEndingAngleDegrees);
		Logger.recordOutput("[CMD_TeleopAlign] PID Output", -output);

		if (Math.abs(setpointDegrees - optimalEndingAngleDegrees) < 2.0) {
			isCommandDone = true;
		}
	}

	// Command ends immediately
	@Override
	public boolean isFinished() {
		return isCommandDone || manualCancel.get();
	}
}
