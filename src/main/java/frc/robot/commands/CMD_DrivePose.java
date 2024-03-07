// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SUB_Swerve;
import java.util.function.Supplier;

public class CMD_DrivePose extends Command {

	private SUB_Swerve swerve;
	private Pose2d pose;

	private PIDController xPID;
	private PIDController yPID;
	private PIDController rPID;

	private Supplier<Boolean> manualCancel;

	public CMD_DrivePose(SUB_Swerve swerve, Pose2d pose, Supplier<Boolean> manualCancel) {
		this.swerve = swerve;
		this.pose = pose;
		addRequirements(swerve);

		xPID = new PIDController(0.1, 0, 0);
		yPID = new PIDController(0.1, 0, 0);
		rPID = new PIDController(0.004, 0, 0);
		rPID.enableContinuousInput(0, 360);
		this.manualCancel = manualCancel;
	}

	// Print a message to the driver station and set the arm state
	@Override
	public void initialize() {}

	@Override
	public void execute() {
		Pose2d currentPose = swerve.getPose();

		double speedX = xPID.calculate(currentPose.getX(), pose.getX());
		double speedY = yPID.calculate(currentPose.getY(), pose.getY());
		double speedR =
				rPID.calculate(currentPose.getRotation().getDegrees(), pose.getRotation().getDegrees());

		swerve.driveRaw(-speedX, speedY, -speedR);
	}

	// Command ends immediately
	@Override
	public boolean isFinished() {
		return manualCancel.get();
	}
}
