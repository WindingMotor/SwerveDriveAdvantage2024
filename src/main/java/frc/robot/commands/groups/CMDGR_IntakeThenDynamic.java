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
import frc.robot.Constants.States.ArmState;
import frc.robot.Constants.States.ShooterMode;
import frc.robot.Constants.States.ShooterState;
import frc.robot.commands.CMD_Align;
import frc.robot.commands.CMD_Idle;
import frc.robot.commands.CMD_IntakeAuto;
import frc.robot.commands.CMD_Led;
import frc.robot.commands.CMD_Shoot;
import frc.robot.subsystems.arm.SUB_Arm;
import frc.robot.subsystems.conveyor.SUB_Conveyor;
import frc.robot.subsystems.shooter.SUB_Shooter;
import frc.robot.subsystems.swerve.SUB_Swerve;
import frc.robot.subsystems.vision.SUB_Vision;
import frc.robot.util.AddressableLedStrip;
import frc.robot.util.AddressableLedStrip.LEDState;
import java.util.function.Supplier;

public class CMDGR_IntakeThenDynamic extends SequentialCommandGroup {

	public CMDGR_IntakeThenDynamic(
			SUB_Swerve swerve,
			SUB_Conveyor conveyor,
			SUB_Arm arm,
			SUB_Shooter shooter,
			SUB_Vision vision,
			AddressableLedStrip led,
			Supplier<Pose2d> robotPose) {
		addRequirements(swerve, conveyor, arm, shooter, vision, led);
		addCommands(
				new CMD_Led(led, LEDState.BLUE),
				new CMD_IntakeAuto(conveyor, arm, () -> false), // Intake the donut
				new CMD_Align(
						swerve, () -> 0.0, () -> 0.0, () -> 0.0, true, () -> false), // Align with speaker
				new CMD_Shoot( // Shoot with dynamic
						conveyor,
						arm,
						shooter,
						vision,
						led,
						ShooterMode.DYNAMIC,
						() -> false,
						() -> false,
						true,
						ShooterState.OFF,
						ArmState.OFF,
						() -> swerve.getPose()),
				new WaitCommand(0.15), // Delay to allow dount to leave the robot
				new CMD_Idle(conveyor, arm, shooter),
				new CMDGR_LedFlash(led, LEDState.GREEN, 5, 0.1),
				new CMD_Led(led, LEDState.RAINBOW));
	}
}
