// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

public class OperatorRumble {

	// private static XboxController operatorController = new XboxController(4);

	public static void rumble(boolean rumble) {
		if (rumble) {
			// operatorController.setRumble(RumbleType.kBothRumble, 0.6);
		} else {
			//	operatorController.setRumble(RumbleType.kBothRumble, 0.0);
		}
	}
}
