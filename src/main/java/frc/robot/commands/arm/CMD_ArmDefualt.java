// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Auto.ScoringPoses;
import frc.robot.Constants.States.ArmState;
import frc.robot.subsystems.arm.SUB_Arm;
import frc.robot.util.MathCalc;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;

public class CMD_ArmDefualt extends Command {

	private SUB_Arm arm;
	private Supplier<Pose2d> robotPose;

	public CMD_ArmDefualt(SUB_Arm arm, Supplier<Pose2d> robotPose) {
		this.arm = arm;
		this.robotPose = robotPose;
		addRequirements(arm);
	}

	// Print a message to the driver station and set the arm state
	@Override
	public void initialize() {}

	@Override
	public void execute() {

		// Blu threshold: 2.5m

		var alli = DriverStation.getAlliance();
		boolean withinArea = false;

		if (alli.get() == Alliance.Blue) {
			if (robotPose.get().getX() < 2.5) {
				withinArea = true;
			}

		} else if (alli.get() == Alliance.Red) {
			withinArea = false;
		}

		Pose2d targetSpeakerPose;

		if (withinArea) {
			if (alli.get() == Alliance.Blue) {
				targetSpeakerPose = ScoringPoses.BLU_SPEAKER.pose;

			} else if (alli.get() == Alliance.Red) {
				targetSpeakerPose = ScoringPoses.RED_SPEAKER.pose;

			} else {
				targetSpeakerPose = new Pose2d();
				DriverStation.reportError("[error] Could not find alliance for auto angle", false);
			}

			double distanceToSpeaker = PhotonUtils.getDistanceToPose(robotPose.get(), targetSpeakerPose);

			double armCalculation = MathCalc.calculateInterpolate(distanceToSpeaker);

			arm.setDynamicAngle(armCalculation);

			Logger.recordOutput("[CMD_Shoot] Dynamic Angle", armCalculation);
		} else {
			arm.setState(ArmState.IDLE);
		}
	}

	@Override
	public void end(boolean interrupted) {}

	// Command ends immediately
	@Override
	public boolean isFinished() {
		return false;
	}
}
