package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class Constants {

    // Current robot mode.
    public static final RobotMode currentMode = RobotMode.SIM;
    public static final double loopPeriodSecs = 0.02;

    public enum RobotMode{
        // Running on a real robot.
        REAL,
        // Running a physics simulator.
        SIM,
        // Replaying from a log file.
        REPLAY
    }

    public enum ModuleSettings{
        FRONTLEFT(1, 2, 0, new Rotation2d(), "FrontLeft"),
        FRONTRIGHT(3, 4, 1, new Rotation2d(), "FrontRight"),
        BACKLEFT(7, 8, 3, new Rotation2d(), "BackLeft"),
        BACKRIGHT(5, 6, 2, new Rotation2d(), "BackRight"),
        DEFAULT(26, 27, 32, new Rotation2d(), "DEFAULT");

        public final int driveID, turnID, absoluteEncoderID;
        public final Rotation2d absoluteEncoderOffset;
        public final String moduleName;

        ModuleSettings(int driveID, int turnID, int absoluteEncoderID, Rotation2d absoluteEncoderOffset, String moduleName) {
            this.driveID = driveID;
            this.turnID = turnID;
            this.absoluteEncoderID = absoluteEncoderID;
            this.absoluteEncoderOffset = absoluteEncoderOffset;
            this.moduleName = moduleName;
        }
      
    }

    public static class MK4SDS{
        public static final double WHEEL_DIAMETER = 0.1016; // 4 in
        public static final double DRIVE_GEAR_RATIO = 1 / 6.12; // L3
        public static final double TURN_GEAR_RATIO = 1 / 12.8;

        public static final double DRIVE_ROT_2_METER = DRIVE_GEAR_RATIO * Math.PI * WHEEL_DIAMETER;
        public static final double TURN_ROT_2_RAD = TURN_GEAR_RATIO * 2 * Math.PI;

        public static final double DRIVE_RPM_2_MPS = DRIVE_ROT_2_METER / 60;
        public static final double TURN_RPM_2_RADPS = TURN_ROT_2_RAD / 60;

        public static final double FREE_MOTOR_SPEED_RPS = 5676 / 60;

        // PIDs
        public static final double TURN_MODULE_PID_P = 0.001; // 0.5
        public static final double TURN_MODULE_PID_I = 0.000001; // 0.0
        public static final double TURN_MODULE_PID_D = 0.00025; // 0.0

        public static final double DRIVE_MODULE_PID_P = 0.45;
        public static final double DRIVE_MODULE_PID_I = 0.0;
        public static final double DRIVE_MODULE_PID_D = 0.0;
    }

    public static class Kinematics{
        public static final double TRACK_WIDTH = 0.53975; // Distance between RIGHT and LEFT wheel centers
        public static final double WHEEL_BASE = 0.53975; // Distance between FRONT and BACK wheel centers

        public static final Translation2d[] MODULE_TRANSLATIONS = {
                new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), // Front Left
                new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), // Front Right
                new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), // Back Left
                new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2) // Back Right
        };

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(MODULE_TRANSLATIONS);

        // Used for conversion to M/S
        public static final double MAX_LINEAR_VELOCITY = 12.0;
        public static final double MAX_ANGULAR_VELOCITY = 12.0;

        // Joystick speed limits
        public static final double LINEAR_SPEED_LIMIT = 12.0;
        public static final double ANGULAR_SPEED_LIMIT = 12.0;

        public static final double DRIVE_DEADBAND = 0.05;
    }

    public enum DriverBindings{
        TX16S(0, false, 1, false, 3, false),
        XBOX(1, true, 0, true, 4, false);

        public final int xInput, yInput, rInput;
        public final boolean xInverted, yInverted, rInverted;

        DriverBindings(int xInput, boolean xInverted, int yInput, boolean yInverted, int rInput, boolean rInverted) {
            this.xInput = xInput;
            this.xInverted = xInverted;
            this.yInput = yInput;
            this.yInverted = yInverted;
            this.rInput = rInput;
            this.rInverted = rInverted;
        }
    }
    
}
