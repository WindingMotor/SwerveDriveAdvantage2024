// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.sidekick;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Auto.SidekickState;
import frc.robot.Constants.States.ArmState;
import frc.robot.subsystems.arm.SUB_Arm;
import frc.robot.subsystems.conveyor.SUB_Conveyor;
import frc.robot.subsystems.shooter.SUB_Shooter;
import frc.robot.subsystems.swerve.SUB_Swerve;
import frc.robot.subsystems.vision.SUB_Vision;
import frc.robot.util.AddressableLedStrip;

// The sidekick automatically schedules commands for the robots subsystems depending on where the
// robot is on the field
public class SUB_Sidekick extends SubsystemBase {

	private final SUB_Swerve swerve;
	private final SUB_Vision vision;
	private final SUB_Arm arm;
	private final SUB_Conveyor conveyor;
	private final SUB_Shooter shooter;
	private final AddressableLedStrip led;
	private final CommandXboxController operatorController;

	private final boolean SIDEKICK_ENABLED = true;
	private SidekickState state;
	private SidekickState lastState;

	public SUB_Sidekick(
			SUB_Swerve swerve,
			SUB_Vision vision,
			SUB_Arm arm,
			SUB_Conveyor conveyor,
			SUB_Shooter shooter,
			AddressableLedStrip led,
			CommandXboxController operatorController) {
		this.swerve = swerve;
		this.vision = vision;
		this.arm = arm;
		this.conveyor = conveyor;
		this.shooter = shooter;
		this.led = led;
		this.operatorController = operatorController;
		state = SidekickState.NONE;
		lastState = SidekickState.UNKNOWN;
	}

	@Override
	public void periodic() {

		if (SIDEKICK_ENABLED
				&& DriverStation.isEnabled()
				&& DriverStation.isTeleopEnabled()
				&& !DriverStation.isAutonomous()) {

			// Operator controller rumble
			if (conveyor.inputs.indexerInitalSensorState) {
				operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.6);
			} else {
				operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
			}

			Pose2d robotPose = swerve.getPose();
			ArmState lastArmState = arm.getState();

			var alli = DriverStation.getAlliance();
			CommandScheduler commandScheduler = CommandScheduler.getInstance();

			/*
				// Blue Alliance
				if (alli.get() == DriverStation.Alliance.Blue) {

					// Blu Speaker
					if (isWithinRectangle(robotPose, 0, 7.0, 1.5, 4.0) && lastArmState != ArmState.INTAKE) {
						commandScheduler.schedule(
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
					}

					// Red Alliance
				} else if (alli.get() == DriverStation.Alliance.Red) {

					// Red Speaker
					if (isWithinRectangle(robotPose, 16.5, 7.0, 15.0, 4.0) && lastArmState != ArmState.INTAKE) {
						commandScheduler.schedule(
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
					}
				}
			*/
		}
	}

	public SidekickState getState() {
		return state;
	}

	public boolean isWithinRectangle(Pose2d currentPose, double x1, double y1, double x2, double y2) {
		double currentX = currentPose.getX();
		double currentY = currentPose.getY();
		return currentX >= x1 && currentX <= x2 && currentY >= y1 && currentY <= y2;
	}
}
