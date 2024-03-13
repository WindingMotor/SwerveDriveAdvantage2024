// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants;
import frc.robot.Constants.Auto.DriveScoringPoseState;
import frc.robot.commands.CMD_ButtonCancel;
import frc.robot.subsystems.swerve.SUB_Swerve;
import java.util.function.Supplier;

public class CMDGR_DriveToScoringPose extends ParallelRaceGroup {

	/*
	 * Drives the robt to a specified scoring position. This command is a race group and also runs a manual button cancel.
	 */
	public CMDGR_DriveToScoringPose(
			SUB_Swerve swerve, DriveScoringPoseState drivePoseState, Supplier<Boolean> cancel) {

		if (Constants.TELEOP_AUTO_DRIVE_ENABLED) {

			if (drivePoseState == DriveScoringPoseState.SPEAKER) {
				addCommands(swerve.driveToSpeaker(), new CMD_ButtonCancel(cancel));
			} else if (drivePoseState == DriveScoringPoseState.AMP) {
				addCommands(swerve.driveToAmp(), new CMD_ButtonCancel(cancel));
			} else {
				addCommands(new PrintCommand("[error] Unknown pose state"));
			}
		}
	}
}
