// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.States.ShooterMode;
import frc.robot.auto.AutoSelector;
import frc.robot.auto.CommandRegistrar;
import frc.robot.commands.CMDGR_Intake;
import frc.robot.commands.CMDGR_Shoot;
import frc.robot.commands.CMD_ResetOdo;
import frc.robot.commands.CMD_Servo;
import frc.robot.subsystems.arm.IO_ArmReal;
import frc.robot.subsystems.arm.SUB_Arm;
import frc.robot.subsystems.conveyor.IO_ConveyorReal;
import frc.robot.subsystems.conveyor.SUB_Conveyor;
import frc.robot.subsystems.shooter.IO_ShooterReal;
import frc.robot.subsystems.shooter.SUB_Shooter;
import frc.robot.subsystems.sidekick.SUB_Sidekick;
import frc.robot.subsystems.swerve.IO_SwerveReal;
import frc.robot.subsystems.swerve.SUB_Swerve;
import frc.robot.subsystems.vision.IO_VisionReal;
import frc.robot.subsystems.vision.SUB_Vision;
import frc.robot.util.AddressableLedStrip;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {

	private final CommandXboxController driverController = new CommandXboxController(3);
	private final CommandXboxController operatorController = new CommandXboxController(4);

	private final SUB_Vision vision = new SUB_Vision(new IO_VisionReal());

	// All methods using these subsystems should be called in this order -> conveyor, arm, shooter
	private final SUB_Conveyor conveyor = new SUB_Conveyor(new IO_ConveyorReal());

	private final SUB_Arm arm = new SUB_Arm(new IO_ArmReal());

	private final SUB_Shooter shooter = new SUB_Shooter(new IO_ShooterReal());

	private final AddressableLedStrip led = new AddressableLedStrip(0, 68);

	private final SUB_Swerve swerve = new SUB_Swerve(new IO_SwerveReal(), vision);

	// private final SUB_Climb climb = new SUB_Climb(new IO_ClimbReal());

	private final CommandRegistrar commandRegistrar =
			new CommandRegistrar(vision, swerve, conveyor, arm, shooter, led);

	private final SUB_Sidekick sidekick =
			new SUB_Sidekick(swerve, vision, arm, conveyor, shooter, led, operatorController);

	private final AutoSelector autoSelector;

	public RobotContainer() {

		commandRegistrar.register();

		autoSelector = new AutoSelector();

		configureBindings();

		logMetadata();

		swerve.setDefaultCommand(
				swerve.driveJoystick(
						() -> driverController.getRawAxis(1),
						() -> driverController.getRawAxis(0),
						() -> driverController.getRawAxis(3)));
	}

	/** Configure the bindings for the controller buttons to specific commands. */
	private void configureBindings() {

		// Shoot command
		operatorController
				.x()
				.onTrue(
						new CMDGR_Shoot(
								conveyor,
								arm,
								shooter,
								vision,
								led,
								ShooterMode.SPEAKER,
								() -> operatorController.b().getAsBoolean(),
								() -> operatorController.x().getAsBoolean()));

		// Amp command
		operatorController
				.y()
				.onTrue(
						new CMDGR_Shoot(
								conveyor,
								arm,
								shooter,
								vision,
								led,
								ShooterMode.AMP,
								() -> operatorController.b().getAsBoolean(),
								() -> operatorController.y().getAsBoolean()));

		// Trap command
		/*
		operatorController
				.rightStick()
				.onTrue(
						new CMDGR_Shoot(
								conveyor,
								arm,
								shooter,
								vision,
								led,
								ShooterMode.TRAP,
								() -> operatorController.b().getAsBoolean(),
								() -> operatorController.rightStick().getAsBoolean()));
		*/

		// Intake command
		operatorController
				.a()
				.onTrue(new CMDGR_Intake(conveyor, arm, led, () -> operatorController.b().getAsBoolean()));

		// Eject command

		/*
				operatorController
						.rightBumper()
						.onTrue(new CMD_Eject(conveyor, arm, () -> operatorController.b().getAsBoolean()));
		*/

		operatorController.rightStick().onTrue(new CMD_ResetOdo(swerve));

		// Climb command, requires both operator and driver to activate
		// operatorController.leftBumper().debounce(0.15).onTrue(new CMD_ArmClimb(arm));

		operatorController.rightBumper().onTrue(new CMD_Servo(arm));

		/*
		// Drive to speaker command
		operatorController
				.leftStick()
				.debounce(0.15)
				.onTrue(
						new CMDGR_DriveToScoringPose(
								swerve,
								DriveScoringPoseState.SPEAKER,
								() -> operatorController.b().getAsBoolean()));
		*/
	}

	// 041215128954433
	public void logMetadata() {
		Logger.recordMetadata("Event Name", DriverStation.getEventName());
		// Logger.recordMetadata("Alliance", DriverStation.getAlliance().get().toString());
		Logger.recordMetadata("Driver Station Location", DriverStation.getLocation() + "");
		Logger.recordMetadata("Match Number", DriverStation.getMatchNumber() + "");
		Logger.recordMetadata("Match Type", DriverStation.getMatchType() + "");
		Logger.recordMetadata("Replay Number", DriverStation.getReplayNumber() + "");
	}

	/**
	 * Get the autonomous command.
	 *
	 * @return The autonomous command
	 */
	public Command getAutonomousCommand() {
		return autoSelector.getSelectedAuto();
	}
}
