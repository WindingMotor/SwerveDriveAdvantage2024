// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.CMD_Led;
import frc.robot.util.AddressableLedStrip;
import frc.robot.util.AddressableLedStrip.LEDState;

public class CMDGR_LedFlash extends SequentialCommandGroup {

	public CMDGR_LedFlash(AddressableLedStrip led, LEDState state, int amount, Double delaySeconds) {
		addRequirements(led);

		// Loop through amount and add the led commands
		for (int i = 0; i < amount; i++) {
			addCommands(
					new CMD_Led(led, state), new WaitCommand(delaySeconds), new CMD_Led(led, LEDState.OFF));
		}
	}
}
