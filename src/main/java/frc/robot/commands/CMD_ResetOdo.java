// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SUB_Swerve;

public class CMD_ResetOdo extends Command {

	private SUB_Swerve swerve;

	public CMD_ResetOdo(SUB_Swerve swerve) {
		this.swerve = swerve;
		addRequirements(swerve);
	}

	// Print a message to the driver station and set the arm state
	@Override
	public void initialize() {
		// Stop and idle the robot subsystems
		swerve.resetEmergencyOdometry();
	}

	// Command ends immediately
	@Override
	public boolean isFinished() {
		return true;
	}
}
