// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands.CMDFL_swerve;

// Deprecated, will be removed in the future! Use CMD_RotationOverride instead.
/*
public class DEP_CMD_AlignAuto extends Command {

	private final SUB_Swerve swerve;
	private final PIDController pid;

	private boolean isCommandDone;
	private int timer = 0;
	private boolean isDriverControlled;

	private Supplier<Double> xInput;
	private Supplier<Double> yInput;

	public DEP_CMD_AlignAuto(
			SUB_Swerve swerve,
			boolean isDriverControlled,
			Supplier<Double> xInput,
			Supplier<Double> yInput) {
		this.swerve = swerve;
		this.isDriverControlled = isDriverControlled;
		this.xInput = xInput;
		this.yInput = yInput;

		addRequirements(swerve);
		this.pid =
				new PIDController(
						Auto.SWERVE_ALIGN_PID.kP, Auto.SWERVE_ALIGN_PID.kI, Auto.SWERVE_ALIGN_PID.kD);
		this.pid.enableContinuousInput(-180, 180);
		this.pid.setTolerance(1.0);
	}

	// Print a message to the driver station and idle the robot subsystems
	@Override
	public void initialize() {
		isCommandDone = false;
		pid.reset();
	}

	@Override
	public void execute() {

		timer++;

		// Get allaince
		var alli = DriverStation.getAlliance();
		Pose2d targetSpeakerPose;

		// Assign target speaker pose based on the alliance
		if (alli.get() == Alliance.Blue) {
			targetSpeakerPose = ScoringPoses.BLU_SPEAKER.pose;
		} else if (alli.get() == Alliance.Red) {
			targetSpeakerPose = ScoringPoses.RED_SPEAKER.pose;
		} else {
			targetSpeakerPose = new Pose2d();
			DriverStation.reportError("[error] Could not find alliance for auto angle", false);
		}

		// Find X and H distances for trig on finding the angle to rotate to
		double xDistanceMeters = swerve.getPose().getX();
		double hDistanceMeters = PhotonUtils.getDistanceToPose(swerve.getPose(), targetSpeakerPose);

		// Find Y distance for applying top / botton angles of the speaker
		double yDistanceMeters = swerve.getPose().getY();

		double setpointRadians = Math.toRadians(180);

		double optimalEndingAngle = 0;

		// If the alliance is blue
		if (alli.get() == Alliance.Blue) {

			// If robot is ABOVE the amp with a middle tolerance of 0.5 meters
			if (yDistanceMeters > targetSpeakerPose.getY()) {
				setpointRadians =
						Math.toRadians(90)
								- (Math.asin(xDistanceMeters / hDistanceMeters))
								+ Math.toRadians(180);
				optimalEndingAngle = swerve.getPose().getRotation().getDegrees() + 360;

				// If robot is BELOW the amp with a middle tolerance of 0.5 meters
			} else if (yDistanceMeters < targetSpeakerPose.getY()) {
				setpointRadians = (Math.toRadians(90) + (Math.asin(xDistanceMeters / hDistanceMeters)));
				optimalEndingAngle = swerve.getPose().getRotation().getDegrees();
			}

			// If the alliance is red
		} else if (alli.get() == Alliance.Red) {

			// Apply offset to the xDistanceMeters beacuse the measurements are taken off the opposite
			// side of the field
			xDistanceMeters = ScoringPoses.RED_SPEAKER.pose.getX() - swerve.getPose().getX();

			// If robot is ABOVE the speaker with a middle tolerance of 0.5 meters
			if (yDistanceMeters > targetSpeakerPose.getY() + 0.25) {
				setpointRadians = (Math.asin(xDistanceMeters / hDistanceMeters)) - Math.toRadians(270);

				// If robot is BELOW the speaker with a middle tolerance of 0.5 meters
			} else if (yDistanceMeters < targetSpeakerPose.getY() - 0.25) {
				setpointRadians = (Math.asin(xDistanceMeters / hDistanceMeters)) + Math.toRadians(135);
			}
		}

		Logger.recordOutput("[CMD_Align] H Distance", hDistanceMeters);
		Logger.recordOutput("[CMD_Align] X Distance", xDistanceMeters);
		Logger.recordOutput("[CMD_Align] Calculated Angle", setpointRadians);
		Logger.recordOutput(
				"[CMD_Align] Desired Pose",
				new Pose2d(swerve.getPose().getTranslation(), new Rotation2d(setpointRadians)));

		Logger.recordOutput("[CMD_Align] Real Angle OPTIMAL", optimalEndingAngle);
		Logger.recordOutput("[CMD_Align] Desired Angle", Math.toDegrees(setpointRadians));

		Logger.recordOutput(
				"[CMD_Align] Desired vs Real Difference",
				setpointRadians - swerve.getPose().getRotation().getDegrees());

		Logger.recordOutput("[CMD_Align] Is Done? ", isCommandDone);

		double output =
				pid.calculate(swerve.getPose().getRotation().getDegrees(), Math.toDegrees(setpointRadians));

		/*
		 * gyro based
		double output =
				pid.calculate(
						swerve.getYaw().getDegrees(), Math.toDegrees(setpointRadians));


		if (isDriverControlled) {
		//	swerve.driveJoystickHybrid(xInput.get(), yInput.get(), output);
		} else {
			//swerve.driveRaw(0.0, 0.0, output);
		}

		swerve.drive(xInput.get(), yInput.get(), output, null, null)

		// End command if the robot is aligned and within 3 degrees

		if (Math.abs(Math.toDegrees(setpointRadians) - optimalEndingAngle) < 4.5) {
			isCommandDone = true;
		}
	}

	@Override
	public void end(boolean interrupted) {
		swerve.driveRaw(0.0, 0.0, 0.0);
	}

	@Override
	public boolean isFinished() {
		return isCommandDone;
	}
}
*/
