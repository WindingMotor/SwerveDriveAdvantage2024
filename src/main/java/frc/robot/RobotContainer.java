// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotMode;
import frc.robot.Constants.States.ShooterMode;
import frc.robot.auto.AutoSelector;
import frc.robot.auto.CommandRegistrar;
import frc.robot.commands.CMDFL_climb.CMD_Climb;
import frc.robot.commands.CMDFL_groups.CMDGR_Intake;
import frc.robot.commands.CMDFL_groups.CMDGR_Shoot;
import frc.robot.commands.CMDFL_groups.CMDGR_ShootOLDAMP;
import frc.robot.commands.CMDFL_groups.CMDGR_TeleopDynamic;
import frc.robot.commands.CMDFL_intake.CMD_Eject;
import frc.robot.commands.CMDFL_intake.CMD_IntakeSource;
import frc.robot.subsystems.arm.IO_ArmReal;
import frc.robot.subsystems.arm.IO_ArmSim;
import frc.robot.subsystems.arm.SUB_Arm;
import frc.robot.subsystems.climb.IO_ClimbReal;
import frc.robot.subsystems.climb.SUB_Climb;
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

	private final CommandXboxController driverController = new CommandXboxController(0);
	private final CommandXboxController operatorController = new CommandXboxController(1);
	private final CommandJoystick climbController = new CommandJoystick(2);

	private final SUB_Vision vision = new SUB_Vision(new IO_VisionReal());

	private final SUB_Conveyor conveyor = new SUB_Conveyor(new IO_ConveyorReal());

	private final SUB_Arm arm =
			(Constants.CURRENT_MODE == RobotMode.SIM)
					? new SUB_Arm(new IO_ArmSim())
					: new SUB_Arm(new IO_ArmReal());

	private final SUB_Shooter shooter = new SUB_Shooter(new IO_ShooterReal());

	private final AddressableLedStrip led = new AddressableLedStrip(1, 64);

	private final SUB_Swerve swerve = new SUB_Swerve(new IO_SwerveReal(), vision, driverController);

	private final SUB_Climb climb = new SUB_Climb(new IO_ClimbReal());

	private final CommandRegistrar commandRegistrar =
			new CommandRegistrar(vision, swerve, conveyor, arm, shooter, led);

	private final SUB_Sidekick sidekick =
			new SUB_Sidekick(swerve, vision, arm, conveyor, shooter, led, operatorController);

	private final AutoSelector autoSelector;

	public RobotContainer() {

		commandRegistrar.register();

		autoSelector = new AutoSelector();

		configureDefaultCommands();
		configureOperatorCommands();
		configureClimbCommands();

		logMetadata();
	}

	private void configureDefaultCommands() {

		swerve.setDefaultCommand(
				swerve.drive(
						() -> driverController.getRawAxis(1),
						() -> driverController.getRawAxis(0),
						() -> driverController.getRawAxis(3)));

		if (Constants.ENABLE_DANGEROUS_DEFAULT_COMMANDS) {
			// arm.setDefaultCommand(new CMD_ArmDefualt(arm, () -> swerve.getPose()));
			sidekick.start();
		}
	}

	private void configureOperatorCommands() {

		// Standard shoot, for being lined up with speaker
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
								() -> operatorController.x().getAsBoolean(),
								() -> swerve.getPose()));

		// Dynamic shoot, auto angles robot and shooter
		operatorController
				.leftBumper()
				.onTrue(
						new CMDGR_TeleopDynamic(
								swerve,
								() -> driverController.getRawAxis(0),
								() -> driverController.getRawAxis(1),
								() -> driverController.getRawAxis(3),
								conveyor,
								arm,
								shooter,
								vision,
								led,
								() -> operatorController.b().getAsBoolean(),
								() -> operatorController.leftBumper().getAsBoolean(),
								() -> swerve.getPose()));

		// Amp scoring
		operatorController
				.y()
				.onTrue(
						new SequentialCommandGroup(
								new CMDGR_Shoot(
										conveyor,
										arm,
										shooter,
										vision,
										led,
										ShooterMode.AMP,
										() -> operatorController.b().getAsBoolean(),
										() -> operatorController.y().getAsBoolean(),
										() -> swerve.getPose())));

		// AMP OLD
		operatorController
				.rightStick()
				.onTrue(
						new SequentialCommandGroup(
								new CMDGR_ShootOLDAMP(
										conveyor,
										arm,
										shooter,
										vision,
										led,
										ShooterMode.AMP,
										() -> operatorController.b().getAsBoolean(),
										() -> operatorController.rightStick().getAsBoolean(),
										() -> swerve.getPose())));

		// Intaking, from ground
		operatorController
				.a()
				.onTrue(new CMDGR_Intake(conveyor, arm, led, () -> operatorController.b().getAsBoolean()));

		// Intaking, from source
		operatorController
				.leftStick()
				.onTrue(
						new CMD_IntakeSource(
								conveyor, arm, shooter, () -> operatorController.b().getAsBoolean()));

		// Eject
		operatorController
				.rightBumper()
				.onTrue(new CMD_Eject(conveyor, arm, () -> operatorController.b().getAsBoolean()));

		/* Drive to BLU amp, look at the method driveToAmp for red alliance stuff.
		THIS METHOD IS CAUSING MEMORY CRASHES.
		operatorController
				.rightStick()
				.onTrue(swerve.driveToPose(Constants.Auto.ScoringPoses.BLU_AMP.pose));
		*/
	}

	private void configureClimbCommands() {

		// Climb command, requires other climb button to be pressed to send
		climbController
				.button(1)
				.onTrue(
						new CMD_Climb(
								led, climb, arm, () -> 1.0, () -> climbController.button(2).getAsBoolean()));

		// operatorController.rightStick().onTrue(new CMD_Servo(arm, true));
	}

	public void logMetadata() {
		Logger.recordMetadata("Event Name", DriverStation.getEventName());
		Logger.recordMetadata("Driver Station Location", DriverStation.getLocation() + "");
		Logger.recordMetadata("Match Number", DriverStation.getMatchNumber() + "");
		Logger.recordMetadata("Match Type", DriverStation.getMatchType() + "");
		Logger.recordMetadata("Replay Number", DriverStation.getReplayNumber() + "");
		Logger.recordMetadata("Robot Mode", "" + Constants.CURRENT_MODE);
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
