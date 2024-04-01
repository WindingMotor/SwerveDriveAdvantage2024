// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.CMDFL_swerve;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.RotationOverrideState;

public class CMD_RotationOverride extends Command {

	private final RotationOverrideState newState;

	public CMD_RotationOverride(RotationOverrideState newState) {
		this.newState = newState;
		addRequirements();
	}

	@Override
	public void initialize() {
		Constants.ROTATION_OVERRIDE_STATE = newState;
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
