// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.SUB_Arm;
import frc.robot.util.AddressableLedStrip;
import frc.robot.util.AddressableLedStrip.LEDState;

public class CMD_ArmClimb extends Command {

	private SUB_Arm arm;
	private boolean isCommandDone = false;
	private int timer = 0;
	private final AddressableLedStrip led;

	public CMD_ArmClimb(SUB_Arm arm, AddressableLedStrip led) {
		this.arm = arm;
		this.led = led;
		addRequirements(arm);
	}

	// Print a message to the driver station and set the arm state
	@Override
	public void initialize() {
		// Stop and idle the robot subsystems
		arm.setClimbMode(true);
		isCommandDone = false;
		timer = 0;
		led.setState(LEDState.BLUE);
	}

	@Override
	public void execute() {
		timer++;
		if (arm.getRealTimeArmPosition() < 0.0 && timer > 50) {
			arm.lockArm();
			isCommandDone = true;
			led.setState(LEDState.AMERICAN);
		} else {
			arm.setSpeed(0.75);
		}
	}

	@Override
	public void end(boolean interrupted) {
		arm.setSpeed(0.0);
	}

	// Command ends immediately
	@Override
	public boolean isFinished() {
		return isCommandDone;
	}
}
