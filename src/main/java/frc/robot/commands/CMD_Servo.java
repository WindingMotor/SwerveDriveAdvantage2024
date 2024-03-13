// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.SUB_Arm;

public class CMD_Servo extends Command {

	private SUB_Arm arm;
	private boolean lock;

	public CMD_Servo(SUB_Arm arm, boolean lock) {
		this.arm = arm;
		this.lock = lock;
		addRequirements(arm);
	}

	// Print a message to the driver station and set the arm state
	@Override
	public void initialize() {
		// Stop and idle the robot subsystems
		arm.setClimbMode(true);
		if (lock) {
			arm.lockArm();
		} else {
			arm.unlockArm();
		}
	}

	@Override
	public void execute() {}

	@Override
	public void end(boolean interrupted) {}

	// Command ends immediately
	@Override
	public boolean isFinished() {
		return true;
	}
}
