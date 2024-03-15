// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.States.ArmState;
import frc.robot.Constants.States.ShooterState;
import frc.robot.subsystems.arm.SUB_Arm;
import frc.robot.subsystems.shooter.SUB_Shooter;

public class CMD_ArmClimbRaise extends Command {

	private SUB_Arm arm;
	private SUB_Shooter shooter;

	public CMD_ArmClimbRaise(SUB_Arm arm, SUB_Shooter shooter) {
		this.arm = arm;
		this.shooter = shooter;
		addRequirements(arm);
	}

	// Print a message to the driver station and set the arm state
	@Override
	public void initialize() {
		arm.setState(ArmState.SPEAKER_1M);
		shooter.setState(ShooterState.OFF);
		arm.setClimbMode(false);
	}

	// Command ends immediately
	@Override
	public boolean isFinished() {
		return true;
	}
}
