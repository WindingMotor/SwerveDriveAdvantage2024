// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Auto.ScoringPoses;
import frc.robot.Constants.States.ArmState;
import frc.robot.Constants.States.ShooterMode;
import frc.robot.Constants.States.ShooterState;
import frc.robot.subsystems.arm.SUB_Arm;
import frc.robot.subsystems.conveyor.SUB_Conveyor;
import frc.robot.subsystems.shooter.SUB_Shooter;
import frc.robot.subsystems.vision.SUB_Vision;
import frc.robot.util.AddressableLedStrip;
import frc.robot.util.AddressableLedStrip.LEDState;
import frc.robot.util.MathCalc;
import frc.robot.util.SwerveAlign;
import java.util.function.Supplier;
import org.photonvision.PhotonUtils;

/** Class to handle shooting commands. */
public class CMD_Shoot extends Command {

	private final SUB_Conveyor conveyor;
	private final SUB_Arm arm;
	private final SUB_Shooter shooter;
	private final AddressableLedStrip led;
	private ShooterMode mode;
	private final Supplier<Boolean> manualCancel;
	private final Supplier<Boolean> shoot;
	private boolean isCommandDone = false;
	private boolean hasShootBeenCalled = false;
	private boolean autoShoot = false;
	private int timer = 0;
	private ShooterState autoShooterState;
	private ArmState autoArmState;
	private Supplier<Pose2d> robotPose;
	private SwerveAlign swerveAlign;

	/**
	 * Constructor for the CMD_Shoot command.
	 *
	 * @param conveyor The conveyor subsystem.
	 * @param arm The arm subsystem.
	 * @param shooter The shooter subsystem.
	 * @param vision The vision subsystem.
	 * @param led The addressable led strip.
	 * @param mode The shooting mode.
	 * @param manualCancel The supplier to determine if the command should be manually cancelled.
	 */
	public CMD_Shoot(
			SUB_Conveyor conveyor,
			SUB_Arm arm,
			SUB_Shooter shooter,
			SUB_Vision vision,
			AddressableLedStrip led,
			ShooterMode mode,
			Supplier<Boolean> manualCancel,
			Supplier<Boolean> shoot,
			Supplier<Pose2d> robotPose,
			SwerveAlign swerveAlign) {
		this.conveyor = conveyor;
		this.arm = arm;
		this.shooter = shooter;
		this.led = led;
		this.mode = mode;
		this.shoot = shoot;
		this.manualCancel = manualCancel;
		this.autoShooterState = null;
		this.autoArmState = null;
		this.robotPose = robotPose;
		this.swerveAlign = swerveAlign;

		addRequirements(conveyor, arm, shooter);
	}

	// Auto Shoot Contructor
	public CMD_Shoot(
			SUB_Conveyor conveyor,
			SUB_Arm arm,
			SUB_Shooter shooter,
			SUB_Vision vision,
			AddressableLedStrip led,
			ShooterMode mode,
			Supplier<Boolean> manualCancel,
			Supplier<Boolean> shoot,
			boolean autoShoot,
			ShooterState autoShooterState,
			ArmState autoArmState,
			SwerveAlign swerveAlign) {
		this.conveyor = conveyor;
		this.arm = arm;
		this.shooter = shooter;
		this.led = led;
		this.mode = mode;
		this.shoot = shoot;
		this.autoShoot = true;
		this.manualCancel = manualCancel;
		this.autoShooterState = autoShooterState;
		this.autoArmState = autoArmState;
		this.swerveAlign = swerveAlign;

		addRequirements(conveyor, arm, shooter);
	}

	// Auto Shoot Contructor
	public CMD_Shoot(
			SUB_Conveyor conveyor,
			SUB_Arm arm,
			SUB_Shooter shooter,
			SUB_Vision vision,
			AddressableLedStrip led,
			ShooterMode mode,
			Supplier<Boolean> manualCancel,
			Supplier<Boolean> shoot,
			boolean autoShoot,
			ShooterState autoShooterState,
			ArmState autoArmState,
			SwerveAlign swerveAlign,
			Supplier<Pose2d> robotPose) {
		this.conveyor = conveyor;
		this.arm = arm;
		this.shooter = shooter;
		this.led = led;
		this.mode = mode;
		this.shoot = shoot;
		this.autoShoot = true;
		this.manualCancel = manualCancel;
		this.autoShooterState = autoShooterState;
		this.autoArmState = autoArmState;
		this.swerveAlign = swerveAlign;
		this.robotPose = robotPose;

		addRequirements(conveyor, arm, shooter);
	}

	/** Reports to the driver station that the command is running and reset all the flags. */
	@Override
	public void initialize() {
		timer = 0;
		isCommandDone = false;
		hasShootBeenCalled = false;
		swerveAlign.setControllerInput(false);
	}

	/** Spool up the shooter to the correct rpm and set arm angle depending on the mode. */
	@Override
	public void execute() {
		setInitalStates();
		checkConveyor();
		checkEndCommand();

		if (manualCancel.get()) {
			isCommandDone = true;
			SmartDashboard.putBoolean("MANUAL CANCEL VAL", manualCancel.get());
		}

		if (mode == ShooterMode.DYNAMIC) {

			double distanceToSpeaker =
					PhotonUtils.getDistanceToPose(robotPose.get(), ScoringPoses.BLU_SPEAKER.pose);

			double armCalculation = MathCalc.calculateInterpolate(distanceToSpeaker);

			shooter.setState(ShooterState.SPEAKER_1M);

			arm.setDynamicAngle(armCalculation);
		}
	}

	private int setInitalStates() {
		// SPEAKER mode
		if (mode == ShooterMode.SPEAKER) {
			shooter.invertMotors(true);
			if (autoShoot) {
				shooter.setState(autoShooterState);
				arm.setState(autoArmState);
			} else {
				shooter.setState(Constants.States.ShooterState.SPEAKER_1M);
				arm.setState(Constants.States.ArmState.SPEAKER_1M);
			}

			// AMP mode
		} else if (mode == ShooterMode.AMP) {
			shooter.invertMotors(true);
			shooter.setState(Constants.States.ShooterState.AMP);
			arm.setState(Constants.States.ArmState.AMP);

			// Trap Mode
		} else if (mode == ShooterMode.TRAP) {

			if (mode == ShooterMode.TRAP) {
				shooter.invertMotors(true);
				if (autoShoot) {
					shooter.setState(autoShooterState);
					arm.setState(autoArmState);
				} else {
					shooter.setState(Constants.States.ShooterState.TRAP);
					arm.setState(Constants.States.ArmState.TRAP);
				}
			}

			// Dyanmic
		} else {

			// Do nothing
		}

		return 1;
	}

	private void checkConveyor() {

		timer++;

		// Manual operator shoot
		if (!autoShoot) {

			if (shooter.isUptoSpeed() && arm.isAtSetpoint()) {
				led.setState(LEDState.WHITE);
			}

			if (shoot.get()) {
				hasShootBeenCalled = true;
				if (mode == ShooterMode.AMP) {
					conveyor.setState(Constants.States.ConveyorState.AMP);
				} else {
					conveyor.setState(Constants.States.ConveyorState.SHOOT);
				}
			}

			// Auto shoot with RPM check
		} else {
			if (shooter.isUptoSpeed() && arm.isAtSetpoint() && timer > 50) {
				led.setState(LEDState.WHITE);
				hasShootBeenCalled = true;
				if (mode == ShooterMode.AMP) {
					conveyor.setState(Constants.States.ConveyorState.AMP);
				} else {
					conveyor.setState(Constants.States.ConveyorState.SHOOT);
				}
			}
		}
	}

	private void checkEndCommand() {

		// Once operator has pressed shoot button and the donut leaves the conveyor end the command
		if (hasShootBeenCalled) {
			if (conveyor.inputs.indexerFinalSensorState) {
				isCommandDone = true;
			}
		}
	}

	/**
	 * Stops and idles the robots subsystems.
	 *
	 * @param interrupted Indicates if the command was interrupted
	 */
	@Override
	public void end(boolean interrupted) {
		led.setState(LEDState.RAINBOW);
		swerveAlign.setControllerInput(true);
	}

	/**
	 * Checks if the command is finished. Can be overriden by manual cancel.
	 *
	 * @return True if the task is finished, false otherwise
	 */
	@Override
	public boolean isFinished() {
		if (manualCancel.get() || isCommandDone) {
			return true;
		} else {
			return false;
		}
	}
}
