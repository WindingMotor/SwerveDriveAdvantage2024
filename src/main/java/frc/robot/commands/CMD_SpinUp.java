// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.States.ShooterState;
import frc.robot.subsystems.shooter.SUB_Shooter;

public class CMD_SpinUp extends Command {

	private SUB_Shooter shooter;

	public CMD_SpinUp(SUB_Shooter shooter) {
		this.shooter = shooter;
		addRequirements(shooter);
	}

	// Print a message to the driver station and set the arm state
	@Override
	public void initialize() {
		shooter.setState(ShooterState.SPEAKER_1M);
	}

	// Command ends immediately
	@Override
	public boolean isFinished() {
		return true;
	}
}
