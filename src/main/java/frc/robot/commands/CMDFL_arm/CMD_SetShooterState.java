// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.CMDFL_arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.States.ShooterState;
import frc.robot.subsystems.shooter.SUB_Shooter;

public class CMD_SetShooterState extends Command {

	private SUB_Shooter shooter;
	private ShooterState state;

	public CMD_SetShooterState(SUB_Shooter shooter, ShooterState state) {
		this.shooter = shooter;
		this.state = state;
		addRequirements(shooter);
	}

	// Print a message to the driver station and set the arm state
	@Override
	public void initialize() {
		// Stop and idle the robot subsystems
		shooter.setState(state);
	}

	// Command ends immediately
	@Override
	public boolean isFinished() {
		return true;
	}
}
