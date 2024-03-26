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
import frc.robot.commands.CMD_DriveAlign;
import frc.robot.commands.CMD_Idle;
import frc.robot.commands.CMD_Led;
import frc.robot.commands.CMD_Shoot;
import frc.robot.subsystems.arm.SUB_Arm;
import frc.robot.subsystems.conveyor.SUB_Conveyor;
import frc.robot.subsystems.shooter.SUB_Shooter;
import frc.robot.subsystems.swerve.SUB_Swerve;
import frc.robot.subsystems.vision.SUB_Vision;
import frc.robot.util.AddressableLedStrip;
import frc.robot.util.AddressableLedStrip.LEDState;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class CMDGR_ShootDynamic extends SequentialCommandGroup {

	public CMDGR_ShootDynamic(
			SUB_Swerve swerve,
			DoubleSupplier xInput,
			DoubleSupplier yInput,
			DoubleSupplier rInput,
			SUB_Conveyor conveyor,
			SUB_Arm arm,
			SUB_Shooter shooter,
			SUB_Vision vision,
			AddressableLedStrip led,
			Supplier<Boolean> manualCancel,
			Supplier<Boolean> shoot,
			Supplier<Pose2d> robotPose) {
		addRequirements(conveyor, arm, shooter);
		// Call the shoot command, then wait 0.5 seconds, then call the idle command
		addCommands(
				new CMD_DriveAlign(swerve, xInput, yInput, rInput, true),
				new CMD_Led(led, LEDState.BLUE),
				new WaitCommand(0.15),
				new CMD_Shoot(
						conveyor,
						arm,
						shooter,
						vision,
						led,
						ShooterMode.DYNAMIC,
						manualCancel,
						shoot,
						robotPose));

		addCommands(
				new WaitCommand(0.15) // Delay to allow dount to leave the robot
				);

		addCommands(
				new CMD_Idle(conveyor, arm, shooter),
				new CMDGR_LedFlash(led, LEDState.GREEN, 5, 0.1),
				new CMD_Led(led, LEDState.RAINBOW));
	}
}
