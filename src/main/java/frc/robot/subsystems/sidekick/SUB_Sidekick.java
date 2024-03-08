// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.sidekick;

/*
The sidekick automatically schedules commands for the robots subsystems depending on where the robot is on the field
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
		if (SIDEKICK_ENABLED && DriverStation.isEnabled() && DriverStation.isTeleopEnabled()) {

			Translation2d robotTranslation = swerve.getPose().getTranslation();

			var alliance = DriverStation.getAlliance();

			// Check if robot is on the blue side or on the red side of the field
			if (alliance.isPresent() && alliance.get() == Alliance.Blue) {

				if (MathCalc.isPointInsideTriangle( // BLU LEFT SIDE OF SPEAKER
						FieldTriangles.BLU_LEFT_SPEAKER.pointOne,
						FieldTriangles.BLU_LEFT_SPEAKER.pointTwo,
						FieldTriangles.BLU_LEFT_SPEAKER.pointThree,
						robotTranslation)) {
					lastState = state;
					state = SidekickState.BLU_LEFT_SPEAKER;

				} else if (MathCalc.isPointInsideTriangle( // BLU RIGHT SIDE OF SPEAKER
						FieldTriangles.BLU_RIGHT_SPEAKER.pointOne,
						FieldTriangles.BLU_RIGHT_SPEAKER.pointTwo,
						FieldTriangles.BLU_RIGHT_SPEAKER.pointThree,
						robotTranslation)) {
					lastState = state;
					state = SidekickState.BLU_RIGHT_SPEAKER;

				} else if (MathCalc.isPointInsideTriangle( // BLU MIDDLE OF SPEAKER
								FieldTriangles.BLU_MIDDLE_SPEAKER_LEFT.pointOne,
								FieldTriangles.BLU_MIDDLE_SPEAKER_LEFT.pointTwo,
								FieldTriangles.BLU_MIDDLE_SPEAKER_LEFT.pointThree,
								robotTranslation)
						|| MathCalc.isPointInsideTriangle(
								FieldTriangles.BLU_MIDDLE_SPEAKER_RIGHT.pointOne,
								FieldTriangles.BLU_MIDDLE_SPEAKER_RIGHT.pointTwo,
								FieldTriangles.BLU_MIDDLE_SPEAKER_RIGHT.pointThree,
								robotTranslation)) {
					lastState = state;
					state = SidekickState.BLU_MIDDLE_SPEAKER;

				} else {
					lastState = state;
					state = SidekickState.NONE;
				}

			} else if (alliance.isPresent() && alliance.get() == Alliance.Red) {

			} else {
				DriverStation.reportError("[error] Sidekick could not determine alliance!", false);
			}

			// Schedule the commands depending on the state
			switch (state) {
				case NONE:
					break;
				case BLU_LEFT_SPEAKER:
					scheduleSpeakerShootCommand();
					break;
				case BLU_MIDDLE_SPEAKER:
					scheduleSpeakerShootCommand();
					break;
				case BLU_RIGHT_SPEAKER:
					scheduleSpeakerShootCommand();
					break;
				case RED_LEFT_SPEAKER:
					break;
				case RED_MIDDLE_SPEAKER:
					break;
				case RED_RIGHT_SPEAKER:
					break;
				default:
					DriverStation.reportError("[error] Sidekick could not determine state!", false);
					break;
			}
		}
	}

	private void scheduleSpeakerShootCommand() {
		// Only schedule the command if the state has changed
		if (state != lastState) {
			System.out.println("SCHEDULING SPEAKER SHOOT");
			CommandScheduler.getInstance()
					.schedule(
							new CMDGR_Shoot(
									conveyor,
									arm,
									shooter,
									vision,
									led,
									ShooterMode.SPEAKER,
									() -> operatorController.b().getAsBoolean(),
									() -> operatorController.x().getAsBoolean()));
		}
	}

	public SidekickState getState() {
		return state;
	}
}

*/
