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
import frc.robot.Constants.RobotMode;
import frc.robot.Constants.States.ShooterMode;
import frc.robot.auto.AutoSelector;
import frc.robot.auto.CommandRegistrar;
import frc.robot.commands.CMD_ArmClimb;
import frc.robot.commands.CMD_Eject;
import frc.robot.commands.groups.CMDGR_Intake;
import frc.robot.commands.groups.CMDGR_Shoot;
import frc.robot.subsystems.arm.IO_ArmReal;
import frc.robot.subsystems.arm.IO_ArmSim;
import frc.robot.subsystems.arm.SUB_Arm;
import frc.robot.subsystems.conveyor.IO_ConveyorReal;
import frc.robot.subsystems.conveyor.SUB_Conveyor;
import frc.robot.subsystems.shooter.IO_ShooterReal;
import frc.robot.subsystems.shooter.SUB_Shooter;
import frc.robot.subsystems.swerve.IO_SwerveReal;
import frc.robot.subsystems.swerve.SUB_Swerve;
import frc.robot.subsystems.vision.IO_VisionReal;
import frc.robot.subsystems.vision.SUB_Vision;
import frc.robot.util.AddressableLedStrip;
import frc.robot.util.OperatorRumble;
import frc.robot.util.SwerveAlign;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {

	private final CommandXboxController driverController = new CommandXboxController(3);
	private final CommandXboxController operatorController = new CommandXboxController(4);

	private final SUB_Vision vision = new SUB_Vision(new IO_VisionReal());

	private final SUB_Conveyor conveyor = new SUB_Conveyor(new IO_ConveyorReal());

	private final SUB_Arm arm =
			(Constants.CURRENT_MODE == RobotMode.SIM)
					? new SUB_Arm(new IO_ArmSim())
					: new SUB_Arm(new IO_ArmReal());

	private final SUB_Shooter shooter = new SUB_Shooter(new IO_ShooterReal());

	private final AddressableLedStrip led = new AddressableLedStrip(0, 40);

	private final SUB_Swerve swerve = new SUB_Swerve(new IO_SwerveReal(), vision);

	// private final SUB_Climb climb = new SUB_Climb(new IO_ClimbReal());

	private final SwerveAlign swerveAlign =
			new SwerveAlign(() -> driverController.getRawAxis(3), () -> swerve.getPose());

	private final CommandRegistrar commandRegistrar =
			new CommandRegistrar(vision, swerve, conveyor, arm, shooter, led, swerveAlign);

	/*
		private final SUB_Sidekick sidekick =
				new SUB_Sidekick(swerve, vision, arm, conveyor, shooter, led, operatorController);
	*/

	private final AutoSelector autoSelector;

	public RobotContainer() {

		commandRegistrar.register();

		autoSelector = new AutoSelector();

		configureBindings();

		logMetadata();

		swerveAlign.setControllerInput(true);

		swerve.setDefaultCommand(
				swerve.driveJoystick(
						() -> driverController.getRawAxis(1),
						() -> driverController.getRawAxis(0),
						() -> swerveAlign.get()));

		// conveyor.setDefaultCommand(new CMD_AutoIntake(conveyor));

		/*
		swerve.setDefaultCommand(
				swerve.driveJoystick(
						() -> -MathUtil.applyDeadband(operatorController.getRawAxis(5), 0.05),
						() -> MathUtil.applyDeadband(operatorController.getRawAxis(4), 0.05),
						() -> MathUtil.applyDeadband(operatorController.getRawAxis(0), 0.05)));
		*/

		OperatorRumble.rumble(false);
	}

	/** Configure the bindings for the controller buttons to specific commands. */
	private void configureBindings() {

		// Shoot command normal
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
								() -> swerve.getPose(),
								swerveAlign));

		// Shoot command dynamic
		operatorController
				.leftBumper()
				.onTrue(
						new CMDGR_Shoot(
								conveyor,
								arm,
								shooter,
								vision,
								led,
								ShooterMode.DYNAMIC,
								() -> operatorController.b().getAsBoolean(),
								() -> operatorController.leftBumper().getAsBoolean(),
								() -> swerve.getPose(),
								swerveAlign));

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
								() -> operatorController.y().getAsBoolean(),
								() -> swerve.getPose(),
								swerveAlign));

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
		operatorController
				.rightBumper()
				.onTrue(new CMD_Eject(conveyor, arm, () -> operatorController.b().getAsBoolean()));

		// operatorController.rightStick().onTrue(new CMD_ResetOdo(swerve));

		// Climb command
		operatorController.leftStick().debounce(0.15).onTrue(new CMD_ArmClimb(arm, led));

		// operatorController.rightStick().debounce(0.1).onTrue(new CMD_Servo(arm, true));
		// operatorController.rightTrigger().debounce(0.15).onTrue(new CMD_Servo(arm, false));

		/*
		operatorController
				.rightBumper()
				.onTrue(
						new CMD_DrivePose(
								swerve,
								new Pose2d(3.4, 5.4, new Rotation2d()),
								() -> operatorController.b().getAsBoolean()));
		*/

		// Drive to speaker command
		/*
		operatorController
				.rightBumper()
				.debounce(0.15)
				.onTrue(
						new CMDGR_DriveToScoringPose(
								swerve, DriveScoringPoseState.AMP, () -> operatorController.b().getAsBoolean()));

								*/
		/*
		operatorController
				.rightBumper()
				.onTrue(
						new CMD_ShootDynamicTest(
								conveyor,
								arm,
								shooter,
								vision,
								led,
								() -> operatorController.b().getAsBoolean(),
								() -> operatorController.rightBumper().debounce(0.1).getAsBoolean(),
								() -> swerve.getPose()));
		*/

		// operatorController.rightBumper().onTrue(new CMD_Arm(arm, ArmState.OFF));

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
