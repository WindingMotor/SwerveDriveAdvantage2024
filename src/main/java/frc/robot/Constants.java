// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/** This class contains constants used in the robot code. */
public class Constants {

	// Current robot mode and loop period.
	public static final RobotMode CURRENT_MODE = RobotMode.REAL;

	// Use extreme caution when enabled, manually controls PID setpoints through smart dashboard.
	public static final boolean PID_TEST_MODE = false;

	// Enables auto driving to a pose with pathplanner during teletop. Be very careful when activated!
	public static final boolean TELEOP_AUTO_DRIVE_ENABLED = true;

	// Enables vision based odometry for swerve drive.
	public static final boolean SWERVE_VISION_ODOMETRY_ENABLED = true;

	// Current robot mode
	public enum RobotMode {
		REAL, // Running on a real robot
		SIM, // Running a physics simulator
		REPLAY // Replaying from a log file
	}

	// Input class for controllers, allows quick hot-swaping.
	public static class Input {

		public static final double DRIVER_DEADBAND = 0.05;
		public static final int DRIVER_PORT = 1;

		public static final double OPERATOR_DEADBAND = 0.05;
		public static final int OPERATOR_PORT = 2;

		public enum InputAxes {
			TX16S(0, false, 1, false, 3, false),
			XBOX(1, true, 0, true, 4, false);
			public final int xInput, yInput, rInput;
			public final boolean xInverted, yInverted, rInverted;

			InputAxes(
					int xInput,
					boolean xInverted,
					int yInput,
					boolean yInverted,
					int rInput,
					boolean rInverted) {
				this.xInput = xInput;
				this.xInverted = xInverted;
				this.yInput = yInput;
				this.yInverted = yInverted;
				this.rInput = rInput;
				this.rInverted = rInverted;
			}
		}
	}

	public static class InputBindings {}

	public static class Vision {
		// The standard deviations of our vision estimated poses, which affect correction rate
		// (Fake values. Experiment and determine estimation noise on an actual robot.)
		// X, Y, Theta
		// public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);
		// public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);

		public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);
		public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.8, 0.8, 1.25);

		public static final double MIN_AMBUGUITY = 0.5;

		// Latency warning trigger threshold in miliseconds
		public static final double LATENCY_THRESHOLD_MILI_SEC = 30.0;

		/*
		Robot Space
		3d Cartesian Coordinate System with (0,0,0) located at the center of the robot’s frame projected down to the floor.
		X+ → Pointing forward (Forward Vector)
		Y+ → Pointing toward the robot’s right (Right Vector)
		Z+ → Pointing upward (Up Vector)
		*/

		// Camera enum, contains name and position of the camera relative to the robot.
		public enum Camera {
			LEFT_CAMERA(
					"OV9281_02",
					"leftCamera",
					new Transform3d(
							// new Translation3d(Units.inchesToMeters(9.558), Units.inchesToMeters(10.9795),
							// Units.inchesToMeters(9.314)), // Camera mounted 12in forward, 9.125in left, 8.625
							// up.
							new Translation3d(
									Units.inchesToMeters(4.558),
									Units.inchesToMeters(21),
									Units.inchesToMeters(
											9.314)), // Camera mounted 12in forward, 9.125in left, 8.625 up.
							new Rotation3d(0, Units.degreesToRadians(-29), 0.0)
									.rotateBy(
											new Rotation3d(
													0.0,
													0.0,
													Units.degreesToRadians(
															-35.5))) // No roll, Pitch on 15 degrees from robot frame, and yaw of
							// 133 degrees from robot frame.
							)),

			RIGHT_CAMERA(
					"OV9281_01",
					"rightCamera",
					new Transform3d(
							new Translation3d(
									Units.inchesToMeters(4.558),
									Units.inchesToMeters(-21),
									Units.inchesToMeters(
											9.314)), // Camera mounted 12in forward, 9.125in right, 8.625 up.
							new Rotation3d(
											0.0, // Roll 0
											Units.degreesToRadians(-29.0), // Pitch FACING UPWARDS
											0.0)
									.rotateBy(
											new Rotation3d(
													0.0, 0.0, Units.degreesToRadians(44.5)) // YAW ROTATING RIGHT IS NEGATIVE
											))
					// new Rotation3d(0,Units.degreesToRadians(29),Units.degreesToRadians(37.5))) // No roll,
					// Pitch on 15 degrees from robot frame, and yaw of 47 degrees from robot frame.
					),

			LIMELIGHT(
					"limelight",
					"limelight",
					new Transform3d(
							new Translation3d(
									Units.inchesToMeters(4.558),
									Units.inchesToMeters(-21),
									Units.inchesToMeters(
											9.314)), // Camera mounted 12in forward, 9.125in right, 8.625 up.
							new Rotation3d(
											0.0, // Roll 0
											Units.degreesToRadians(-29.0), // Pitch FACING UPWARDS
											0.0)
									.rotateBy(
											new Rotation3d(
													0.0, 0.0, Units.degreesToRadians(44.5)) // YAW ROTATING RIGHT IS NEGATIVE
											))
					// new Rotation3d(0,Units.degreesToRadians(29),Units.degreesToRadians(37.5))) // No roll,
					// Pitch on 15 degrees from robot frame, and yaw of 47 degrees from robot frame.
					);

			public final String PHOTON_NAME;
			public final String CAMERA_NAME;

			public final Transform3d ROBOT_TO_CAMERA;

			Camera(String photonName, String cameraName, Transform3d robotToCamera) {
				this.PHOTON_NAME = photonName;
				this.CAMERA_NAME = cameraName;
				this.ROBOT_TO_CAMERA = robotToCamera;
			}
		}
	}

	public static class Maestro {

		// Arm
		public static final int ARM_MOTOR_LEAD_ID = 9;
		public static final int ARM_MOTOR_FOLLOWER_ID = 10;

		public static final boolean ARM_MOTOR_LEAD_INVERTED = false;
		public static final boolean ARM_MOTOR_FOLLOWER_INVERTED = false;

		public static final double ARM_P = 0.135; // Volts 0.45 0.18
		public static final double ARM_I = 0.0;
		public static final double ARM_D = 0.00012;

		public static final double ARM_KS = 0.01; // Volts
		public static final double ARM_KG = 0.379;
		public static final double ARM_KV = 1.86;
		public static final double ARM_KA = 0.06;

		public static final double ARM_MAX_VELOCITY = 460; // Deg/sec
		public static final double ARM_MAX_ACCELERATION = 3000; // Deg/sec/sec

		public static final double ARM_VOLTAGE_CLAMPING = 11.9;

		public static final double ARM_TOLERANCE_DEGREES = 1.0; // 3.0
		public static final double ARM_OFFSET_DEGREES = 4.85;

		// Shooter
		public static final int SHOOTER_MOTOR_BOTTOM_ID = 12;
		public static final int SHOOTER_MOTOR_TOP_ID = 13;

		public static final boolean SHOOTER_MOTOR_BOTTOM_INVERTED = true;
		public static final boolean SHOOTER_MOTOR_TOP_INVERTED = true; // true normal shoot, false amp

		// 6e-5
		public static final double SHOOTER_MOTORS_P = 0.00068;
		public static final double SHOOTER_MOTORS_I = 0.0;
		public static final double SHOOTER_MOTORS_D = 0.00005;
		public static final double SHOOTER_MOTORS_IZ = 0.0005;
		public static final double SHOOTER_MOTORS_FF = 0.00018;

		public static final double SHOOTER_TOLERANCE_RPM = 300;

		// Intake
		public static final int INDEXER_MOTOR_ID = 11;

		// Indexer
		public static final int INTAKE_MOTOR_ID = 14;

		// Sensor IDs
		public static final int INTAKE_SENSOR_ID = 6;

		public static final int INDEXER_INITAL_SENSOR_ID = 2;
		public static final int INDEXER_FINAL_SENSOR_ID = 1;

		public static final int SENSOR_TOLERANCE_MAX_CENTIMETERS = 12;
		public static final int SENSOR_TOLERANCE_MIN_CENTIMETERS = 8;

		// Climb
		public static final int LEFT_CLIMB_MOTOR_ID = 20;
		public static final int RIGHT_CLIMB_MOTOR_ID = 21;

		public static final boolean LEFT_CLIMB_MOTOR_INVERTED = true;
		public static final boolean RIGHT_CLIMB_MOTOR_INVERTED = false;

		/*
		Motor IDs
		1-8 swerve
		9-10 arm
		11 indexer
		12-13 shooter
		*/

	}

	public static class Arduino {

		public static final double PWM_PORT = 5;

		public enum OutputMapping {
			HEARTBEAT(1, "Heartbeat"),
			RED(2, "Red"),
			GREEN(3, "Green"),
			BLUE(4, "Blue"),
			PURPLE(5, "Purple"),
			YELLOW(6, "Yellow");

			public final int rawPWM;
			public final String name;

			OutputMapping(int rawPWM, String name) {
				this.rawPWM = rawPWM;
				this.name = name;
			}
		}
	}

	// States of the robot subsystems
	public static class States {

		// Modes of the shooter
		public enum ShooterMode {
			SPEAKER,
			AMP,
			DYNAMIC,
			TRAP
		}

		// Speeds of the shooter for the various modes
		public enum ShooterState {
			OFF(0.0, 0.0),
			IDLE(3700, 0.0),

			SPEAKER_1M(3600, 1.0),
			SPEAKER_2M(3700, 2.0),
			SPEAKER_2_5M(3700, 2.0),
			SPEAKER_3M(3200, 3.0),
			SPEAKER_4M(3700, 4.0),
			SPEAKER_5M(3700, 5.0),

			TRAP(2500, 0.0),

			AMP(500, 0.0);

			public final double rpm;
			public final double distanceMeters;

			ShooterState(double rpm, double distanceMeters) {
				this.rpm = rpm;
				this.distanceMeters = distanceMeters;
			}
		}

		// Positions of the arm for various modes
		public enum ArmState {
			OFF(0, 0.0),
			IDLE(-4, 0.0),

			SPEAKER_1M(55.0, 1.0),
			SPEAKER_2M(42.0, 2.0), // 43deg
			SPEAKER_2_5M(40, 2.0),
			SPEAKER_3M(38.0, 3.0),
			SPEAKER_4M(34.15, 4.0),

			AMP(95, 0.0),
			DYNAMIC(-1.0, 0.0),
			INTAKE(89.0, 0.0),

			TRAP(61.0, 0.0);

			public final double position;
			public final double distanceMeters;

			ArmState(double position, double distanceMeters) {
				this.position = position;
				this.distanceMeters = distanceMeters;
			}
		}

		// Speeds of conveyor, intake & indexer, for various modes
		public enum ConveyorState {
			OFF(0.0, 0.0),
			INTAKE(-1.0, -0.5),
			EJECT(1.0, 0.45),
			SHOOT(0.0, -1.0),
			AMP(0.0, -0.8);
			public final double intakeSpeed, indexerSpeed;

			ConveyorState(double intakeSpeed, double indexerSpeed) {
				this.intakeSpeed = intakeSpeed;
				this.indexerSpeed = indexerSpeed;
			}
		}
	}

	public static class Auto {

		public static final PIDConstants AUTO_PID = new PIDConstants(0.4, 0.0, 0.05);

		public static final double MAX_VELOCITY_MPS = 1.5;
		public static final double MAX_ACCELERATION_MPS_SQ = 2.0;

		public static final double MAX_ANGULAR_VELOCITY_RADS = 360;
		public static final double MAX_ANGULAR_VELOCITY_RADS_SQ = 540;

		public static final double MAX_MODULE_SPEED_MPS = 4.5;

		public enum ScoringPoses {
			BLU_SPEAKER(new Pose2d(1.5, 5.5, Rotation2d.fromDegrees(180))),
			RED_SPEAKER(new Pose2d(12.0, 5.5, Rotation2d.fromDegrees(180))),

			BLU_AMP(new Pose2d(1.85, 7.75, Rotation2d.fromDegrees(90))),
			RED_AMP(new Pose2d(15.0, 14.75, Rotation2d.fromDegrees(90)));

			public final Pose2d pose;

			ScoringPoses(Pose2d pose) {
				this.pose = pose;
			}
		}

		public enum NotePoses {
			BLU_TOP(new Pose2d(2.9, 7.0, Rotation2d.fromDegrees(0))),
			BLU_MIDDLE(new Pose2d(2.9, 5.5, Rotation2d.fromDegrees(0))),
			BLU_BOTTOM(new Pose2d(2.9, 4.1, Rotation2d.fromDegrees(0))),

			RED_TOP(new Pose2d(13.67, 7.0, Rotation2d.fromDegrees(0))),
			RED_MIDDLE(new Pose2d(13.67, 5.5, Rotation2d.fromDegrees(0))),
			RED_BOTTOM(new Pose2d(13.67, 4.1, Rotation2d.fromDegrees(0)));

			public final Pose2d pose;

			NotePoses(Pose2d pose) {
				this.pose = pose;
			}
		}

		public enum DriveScoringPoseState {
			SPEAKER,
			AMP
		}
	}
}
