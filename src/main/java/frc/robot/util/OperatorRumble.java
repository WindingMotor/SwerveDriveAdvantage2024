// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;

public class OperatorRumble {

	public static void rumble(boolean rumble) {
		XboxController tempOperatorController = new XboxController(4);

		if (rumble && !DriverStation.isAutonomous()) {
			tempOperatorController.setRumble(RumbleType.kBothRumble, 0.6);
		} else {
			tempOperatorController.setRumble(RumbleType.kBothRumble, 0.0);
		}
	}
}
