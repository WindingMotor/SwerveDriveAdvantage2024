// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.auto;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.States.ArmState;
import frc.robot.Constants.States.ShooterMode;
import frc.robot.Constants.States.ShooterState;
import frc.robot.commands.CMD_Eject;
import frc.robot.commands.CMD_Idle;
import frc.robot.commands.CMD_Intake;
import frc.robot.commands.CMD_Led;
import frc.robot.commands.CMD_Shoot;
import frc.robot.subsystems.arm.SUB_Arm;
import frc.robot.subsystems.conveyor.SUB_Conveyor;
import frc.robot.subsystems.shooter.SUB_Shooter;
import frc.robot.subsystems.swerve.SUB_Swerve;
import frc.robot.subsystems.vision.SUB_Vision;
import frc.robot.util.AddressableLedStrip;
import frc.robot.util.AddressableLedStrip.LEDState;

// 041215128954433
public class CommandRegistrar {

	private final SUB_Vision vision;

	private final SUB_Conveyor conveyor;

	private final SUB_Arm arm;

	private final SUB_Shooter shooter;

	private final AddressableLedStrip led;

	public CommandRegistrar(
			SUB_Vision vision,
			SUB_Swerve swerve,
			SUB_Conveyor conveyor,
			SUB_Arm arm,
			SUB_Shooter shooter,
			AddressableLedStrip led) {
		this.vision = vision;
		this.conveyor = conveyor;
		this.arm = arm;
		this.shooter = shooter;
		this.led = led;
	}

	/*
	 * Register all commands for autonomous to be called with pathplanner.
	 * Any manual disabling of commands has been turned off by default.
	 */
	public void register() {

		// Intake command
		NamedCommands.registerCommand("Intake", new CMD_Intake(conveyor, arm, () -> false));

		NamedCommands.registerCommand("Intake_Test", new CMD_Intake(conveyor, arm, () -> false));
		// Eject command
		NamedCommands.registerCommand("Eject", new CMD_Eject(conveyor, arm, () -> false));

		// Null commands for backwards compatibility
		NamedCommands.registerCommand("Shoot_Speaker", new PrintCommand("NULL"));
		NamedCommands.registerCommand("Shoot_Command", new PrintCommand("NULL"));
		NamedCommands.registerCommand("Shoot_Command2M", new PrintCommand("NULL"));
		NamedCommands.registerCommand("Intake_Auto", new PrintCommand("NULL"));

		//
		// Speaker shoot commands for different distances
		// One meter
		NamedCommands.registerCommand(
				"Shoot_Speaker1M",
				new CMD_Shoot(
						conveyor,
						arm,
						shooter,
						vision,
						led,
						ShooterMode.SPEAKER,
						() -> false,
						() -> false,
						true,
						ShooterState.SPEAKER_1M,
						ArmState.SPEAKER_1M));

		// Two Meters
		NamedCommands.registerCommand(
				"Shoot_Speaker2M",
				new CMD_Shoot(
						conveyor,
						arm,
						shooter,
						vision,
						led,
						ShooterMode.SPEAKER,
						() -> false,
						() -> false,
						true,
						ShooterState.SPEAKER_2M,
						ArmState.SPEAKER_2M));

		// Two and a half meters
		NamedCommands.registerCommand(
				"Shoot_Speaker2_5M",
				new CMD_Shoot(
						conveyor,
						arm,
						shooter,
						vision,
						led,
						ShooterMode.SPEAKER,
						() -> false,
						() -> false,
						true,
						ShooterState.SPEAKER_2_5M,
						ArmState.SPEAKER_2_5M));

		// Three meters
		NamedCommands.registerCommand(
				"Shoot_Speaker3M",
				new CMD_Shoot(
						conveyor,
						arm,
						shooter,
						vision,
						led,
						ShooterMode.SPEAKER,
						() -> false,
						() -> false,
						true,
						ShooterState.SPEAKER_3M,
						ArmState.SPEAKER_3M));

		// Amp command
		NamedCommands.registerCommand(
				"Shoot_Amp",
				new CMD_Shoot(
						conveyor,
						arm,
						shooter,
						vision,
						led,
						ShooterMode.AMP,
						() -> false,
						() -> false,
						true,
						ShooterState.AMP,
						ArmState.AMP));

		// Idle command
		NamedCommands.registerCommand("Idle", new CMD_Idle(conveyor, arm, shooter));

		// LED commands
		NamedCommands.registerCommand("Led_Rainbow", new CMD_Led(led, LEDState.RAINBOW));
		NamedCommands.registerCommand("Led_Green", new CMD_Led(led, LEDState.GREEN));
		NamedCommands.registerCommand("Led_Red", new CMD_Led(led, LEDState.RED));
		NamedCommands.registerCommand("Led_Blue", new CMD_Led(led, LEDState.BLUE));
	}
}
