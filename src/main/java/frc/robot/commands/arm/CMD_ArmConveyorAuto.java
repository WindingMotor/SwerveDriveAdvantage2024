// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Auto.ScoringPoses;
import frc.robot.Constants.States.ConveyorState;
import frc.robot.subsystems.arm.SUB_Arm;
import frc.robot.subsystems.conveyor.SUB_Conveyor;
import frc.robot.util.MathCalc;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;

public class CMD_ArmConveyorAuto extends Command {

	private SUB_Arm arm;
	private SUB_Conveyor conveyor;
	private Supplier<Pose2d> robotPose;

	private boolean isCommandDone;
	private Debouncer debouncer;

	public CMD_ArmConveyorAuto(SUB_Arm arm, SUB_Conveyor conveyor, Supplier<Pose2d> robotPose) {
		this.arm = arm;
		this.conveyor = conveyor;
		this.robotPose = robotPose;
		debouncer = new Debouncer(0.025, Debouncer.DebounceType.kRising);
		addRequirements(arm, conveyor);
	}

	// Print a message to the driver station and set the arm state
	@Override
	public void initialize() {
		// Stop and idle the robot subsystems
		isCommandDone = false;
		arm.setClimbMode(false);
		conveyor.setState(ConveyorState.INTAKE);
	}

	@Override
	public void execute() {

		if (debouncer.calculate(conveyor.inputs.indexerInitalSensorState)) {
			isCommandDone = true;
		}

		var alli = DriverStation.getAlliance();
		Pose2d targetSpeakerPose;

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
	}

	@Override
	public void end(boolean interrupted) {
		conveyor.setState(Constants.States.ConveyorState.OFF);
	}

	// Command ends immediately
	@Override
	public boolean isFinished() {
		return isCommandDone;
	}
}
