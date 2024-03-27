// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Auto.ScoringPoses;
import frc.robot.Constants.States.ArmState;
import frc.robot.Constants.States.ConveyorState;
import frc.robot.Constants.States.ShooterMode;
import frc.robot.Constants.States.ShooterState;
import frc.robot.subsystems.arm.SUB_Arm;
import frc.robot.subsystems.conveyor.SUB_Conveyor;
import frc.robot.subsystems.shooter.SUB_Shooter;
import frc.robot.subsystems.vision.SUB_Vision;
import frc.robot.util.AddressableLedStrip;
import frc.robot.util.AddressableLedStrip.LEDState;
import frc.robot.util.MathCalc;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
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

	public CMD_Shoot(
			SUB_Conveyor conveyor,
			SUB_Arm arm,
			SUB_Shooter shooter,
			SUB_Vision vision,
			AddressableLedStrip led,
			ShooterMode mode,
			Supplier<Boolean> manualCancel,
			Supplier<Boolean> shoot,
			Supplier<Pose2d> robotPose) {
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

		addRequirements(conveyor, arm, shooter);
	}

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
			ArmState autoArmState) {
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

		addRequirements(conveyor, arm, shooter);
	}

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
		this.robotPose = robotPose;

		addRequirements(conveyor, arm, shooter);
	}

	/** Reports to the driver station that the command is running and reset all the flags. */
	@Override
	public void initialize() {
		timer = 0;
		isCommandDone = false;
		hasShootBeenCalled = false;
		setInitalStates();
	}

	/** Spool up the shooter to the correct rpm and set arm angle depending on the mode. */
	@Override
	public void execute() {

		// Check if ready to run the conveyor
		checkConveyor();

		// End the command if the flag hasShootBeenCalled is true and the indexer final sensor is
		// triggered
		if (hasShootBeenCalled) {
			if (conveyor.inputs.indexerFinalSensorState && timer > 1) {
				isCommandDone = true;
			}
		}

		// Run dynamic mode if needed
		if (mode == ShooterMode.DYNAMIC) {
			runDynamic();
		}
	}

	/**
	 * Runs the dynamic function to calculate the arm angle based on the alliance color, sets the
	 * shooter state, and records the output.
	 */
	private void runDynamic() {

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

		shooter.setState(ShooterState.SPEAKER_1M);

		arm.setDynamicAngle(armCalculation);

		Logger.recordOutput("[CMD_Shoot] Dynamic Angle", armCalculation);
	}

	/** Set the initial states based on the shooting mode. */
	private void setInitalStates() {

		conveyor.setState(ConveyorState.OFF);

		switch (mode) {
			case SPEAKER:
				setSpeakerState();
				break;

			case AMP:
				setAmpState();
				break;

			default:
				setSpeakerState();
				break;
		}
	}

	/** Sets the speaker state for the shooter and arm based on the autoShoot flag. */
	private void setSpeakerState() {
		shooter.invertMotors(true);
		shooter.setState(autoShoot ? autoShooterState : Constants.States.ShooterState.SPEAKER_1M);
		arm.setState(autoShoot ? autoArmState : Constants.States.ArmState.SPEAKER_1M);
	}

	/** Set the AMP state for the shooter and arm. */
	private void setAmpState() {
		shooter.invertMotors(false);
		shooter.setState(Constants.States.ShooterState.AMP);
		arm.setState(Constants.States.ArmState.AMP);
	}

	/** Check the conveyor status depending on the autoShoot flag. */
	private void checkConveyor() {
		timer++;
		if (autoShoot) {
			checkConveyorAuto();
		} else {
			checkConveyorManual();
		}
	}

	/**
	 * Checks if operator has called the shoot button. Also sets the LED state to white when the
	 * shooter is ready.
	 */
	private void checkConveyorManual() {
		if (shooter.isUptoSpeed() && arm.isAtSetpoint()) {
			led.setState(LEDState.WHITE);
		}

		if (shoot.get() && timer > 30) {
			hasShootBeenCalled = true;
			conveyor.setState(
					mode == ShooterMode.AMP
							? Constants.States.ConveyorState.AMP
							: Constants.States.ConveyorState.SHOOT);
		}
	}

	/** Checks if the shooter is ready and the arm is at the setpoint. Then runs the conveyor. */
	private void checkConveyorAuto() {
		if (shooter.isUptoSpeed() && arm.isAtSetpoint() && timer > 50) {
			led.setState(LEDState.WHITE);
			hasShootBeenCalled = true;
			conveyor.setState(
					mode == ShooterMode.AMP
							? Constants.States.ConveyorState.AMP
							: Constants.States.ConveyorState.SHOOT);
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
