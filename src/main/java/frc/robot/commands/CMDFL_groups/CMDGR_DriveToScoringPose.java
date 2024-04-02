// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.CMDFL_groups;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants.Auto.DriveScoringPoseState;
import frc.robot.subsystems.swerve.SUB_Swerve;
import java.util.function.Supplier;

public class CMDGR_DriveToScoringPose extends ParallelRaceGroup {

	/*
	 * Drives the robot to a specified scoring position. This command is a race group and also runs a manual button cancel.
	 */
	public CMDGR_DriveToScoringPose(
			SUB_Swerve swerve, DriveScoringPoseState drivePoseState, Supplier<Boolean> cancel) {

		/*
		if (Constants.TELEOP_AUTO_DRIVE_ENABLED) {

			if (drivePoseState == DriveScoringPoseState.SPEAKER) {
				addCommands(new ParallelRaceGroup(swerve.driveToSpeaker(), new CMD_Cancel(cancel)));
			} else if (drivePoseState == DriveScoringPoseState.AMP) {
				addCommands(new ParallelRaceGroup(swerve.driveToAmp(), new CMD_Cancel(cancel)));
			} else {
				addCommands(new PrintCommand("[error] Unknown pose state"));
			}
		}
		*/
	}
}
