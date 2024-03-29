// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.CMD_ArmConveyorAuto;
import frc.robot.subsystems.arm.SUB_Arm;
import frc.robot.subsystems.conveyor.SUB_Conveyor;
import java.util.function.Supplier;

public class CMDGR_IntakeDynamicAuto extends SequentialCommandGroup {

	public CMDGR_IntakeDynamicAuto(SUB_Conveyor conveyor, SUB_Arm arm, Supplier<Pose2d> robotPose) {
		addRequirements(conveyor, arm);
		addCommands(new ParallelRaceGroup(new CMD_ArmConveyorAuto(arm, conveyor, robotPose)));
	}
}
