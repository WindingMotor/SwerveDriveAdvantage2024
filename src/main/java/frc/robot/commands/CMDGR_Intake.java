// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.arm.SUB_Arm;
import frc.robot.subsystems.conveyor.SUB_Conveyor;
import frc.robot.util.AddressableLedStrip;
import frc.robot.util.AddressableLedStrip.LEDState;
import java.util.function.Supplier;

public class CMDGR_Intake extends SequentialCommandGroup {

	private final double flashDelaySeconds = 0.1;

	public CMDGR_Intake(
			SUB_Conveyor conveyor, SUB_Arm arm, AddressableLedStrip led, Supplier<Boolean> manualCancel) {
		addRequirements(arm, conveyor);
		// Call intake command and flash leds green
		addCommands(
				new CMD_Intake(conveyor, arm, manualCancel),
				new CMD_Led(led, LEDState.GREEN),
				new WaitCommand(flashDelaySeconds),
				new CMD_Led(led, LEDState.OFF),
				new WaitCommand(flashDelaySeconds),
				new CMD_Led(led, LEDState.GREEN),
				new WaitCommand(flashDelaySeconds),
				new CMD_Led(led, LEDState.OFF),
				new WaitCommand(flashDelaySeconds),
				new CMD_Led(led, LEDState.RAINBOW));
	}
}
