// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.States.ArmState;
import frc.robot.subsystems.arm.SUB_Arm;
import frc.robot.util.AddressableLedStrip;
import java.util.function.Supplier;

public class CMD_ClimbArm extends Command {

	private final AddressableLedStrip led;
	private final SUB_Arm arm;
	private Supplier<Boolean> activate;

	public CMD_ClimbArm(AddressableLedStrip led, SUB_Arm arm, Supplier<Boolean> activate) {
		this.led = led;
		this.arm = arm;
		this.activate = activate;
		addRequirements(led);
	}

	// Print a message to the driver station and set the LED state
	@Override
	public void initialize() {
		arm.setState(ArmState.AMP);
		arm.setCurrentLimits(65);
	}

	@Override
	public void execute() {
		if (activate.get()) {
			arm.setClimbMode(true);
			arm.setSpeed(-1.0);
		}
	}

	// Command ends immediately
	@Override
	public boolean isFinished() {
		return true;
	}
}
