// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.SUB_Arm;
import java.util.function.Supplier;

public class CMD_ArmClimb extends Command {

	private SUB_Arm arm;
	private Supplier<Double> speed;

	public CMD_ArmClimb(SUB_Arm arm, Supplier<Double> speed) {
		this.arm = arm;
		this.speed = speed;
		addRequirements(arm);
	}

	// Print a message to the driver station and set the arm state
	@Override
	public void initialize() {
		// Stop and idle the robot subsystems
		arm.setClimbMode(true);
		arm.setSpeed(speed.get());
	}

	// Command ends immediately
	@Override
	public boolean isFinished() {
		return true;
	}
}
