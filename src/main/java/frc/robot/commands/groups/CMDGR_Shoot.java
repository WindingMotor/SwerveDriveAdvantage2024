// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.States.ShooterMode;
import frc.robot.commands.CMD_Idle;
import frc.robot.commands.CMD_Led;
import frc.robot.commands.CMD_Shoot;
import frc.robot.subsystems.arm.SUB_Arm;
import frc.robot.subsystems.conveyor.SUB_Conveyor;
import frc.robot.subsystems.shooter.SUB_Shooter;
import frc.robot.subsystems.vision.SUB_Vision;
import frc.robot.util.AddressableLedStrip;
import frc.robot.util.AddressableLedStrip.LEDState;
import java.util.function.Supplier;

public class CMDGR_Shoot extends SequentialCommandGroup {

	public CMDGR_Shoot(
			SUB_Conveyor conveyor,
			SUB_Arm arm,
			SUB_Shooter shooter,
			SUB_Vision vision,
			AddressableLedStrip led,
			ShooterMode mode,
			Supplier<Boolean> manualCancel,
			Supplier<Boolean> shoot,
			Supplier<Pose2d> robotPose) {
		addRequirements(conveyor, arm, shooter, vision, led);
		addCommands(
				new CMD_Led(led, LEDState.BLUE),
				new CMD_Shoot(conveyor, arm, shooter, vision, led, mode, manualCancel, shoot, robotPose));

		if (!manualCancel.get()) {
			if (mode == ShooterMode.AMP) {
				addCommands(
						new WaitCommand(1.1) // Longer delay to allow dount to leave the robot
						);
			} else {
				addCommands(
						new WaitCommand(0.15) // Short delay to allow dount to leave the robot
						);
			}
		}

		addCommands(
				new CMD_Idle(conveyor, arm, shooter),
				new CMDGR_LedFlash(led, LEDState.GREEN, 5, 0.1),
				new CMD_Led(led, LEDState.RAINBOW));
	}
}
