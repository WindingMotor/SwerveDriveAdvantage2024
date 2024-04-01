// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.CMDFL_climb;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.States.ArmState;
import frc.robot.subsystems.arm.SUB_Arm;
import frc.robot.subsystems.climb.SUB_Climb;
import frc.robot.util.AddressableLedStrip;
import frc.robot.util.AddressableLedStrip.LEDState;
import java.util.function.Supplier;

public class CMD_Climb extends Command {

	private final AddressableLedStrip led;
	private final SUB_Climb climb;
	private final SUB_Arm arm;
	Supplier<Double> speed;
	Supplier<Boolean> activate;
	boolean send;

	private boolean isCommandDone;

	public CMD_Climb(
			AddressableLedStrip led,
			SUB_Climb climb,
			SUB_Arm arm,
			Supplier<Double> speed,
			Supplier<Boolean> activate) {
		this.led = led;
		this.climb = climb;
		this.arm = arm;
		this.speed = speed;
		this.activate = activate;
		addRequirements(led);
	}

	// Print a message to the driver station and set the LED state
	@Override
	public void initialize() {

		isCommandDone = false;
		send = false;
		led.setState(LEDState.ORANGE);
		arm.setState(ArmState.CLIMB);
	}

	@Override
	public void execute() {

		if (activate.get() && arm.getRealTimeArmPosition() > 25.0) {
			send = true;
		} else {
			DriverStation.reportWarning("[warning] [CMD_Climb] Unable to climb with arm down!", false);
			climb.set(0.0);
			isCommandDone = true;
		}

		if (send) {
			climb.set(speed.get());

			arm.setClimbMode(true);
			// If arm position is less than 30 stop, if not keep running it
			if (arm.getRealTimeArmPosition() - 0.25 < 20) {
				arm.setSpeed(0.0);
				arm.lockArm();
			} else {
				arm.setSpeed(0.65);
			}
		}

		if (climb.inputs.motorPosition > 38.0) {
			climb.set(0);
		}

		if (climb.inputs.motorPosition > 38.0 && arm.getRealTimeArmPosition() - 0.25 < 20) {
			arm.lockArm();
			isCommandDone = true;
		}
	}

	@Override
	public boolean isFinished() {
		return isCommandDone;
	}
}
