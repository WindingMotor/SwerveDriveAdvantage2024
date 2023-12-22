package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * This class contains constants used in the robot code.
*/
public class Constants{

    // Current robot mode and loop period.
    public static final RobotMode CURRENT_MODE = RobotMode.REAL;
    public static final double LOOP_PERIOD_SECS = 0.02;

    /**
     * Enum representing the robot mode.
    */
    public enum RobotMode {
        REAL,   // Running on a real robot
        SIM,    // Running a physics simulator
        REPLAY  // Replaying from a log file
    }

    /**
     * Enum for the settings of each module.
    */
    public enum ModuleSettings {
        FRONT_LEFT(1, true, 2, true, 0, new Rotation2d(0.0), "FrontLeft"),
        FRONT_RIGHT(3, true, 4, true, 1, new Rotation2d(6.11), "FrontRight"),
        BACK_LEFT(7, true, 8, true, 3, new Rotation2d(0.43), "BackLeft"),
        BACK_RIGHT(5, true, 6, true, 2, new Rotation2d(4.53), "BackRight");

        public final int DRIVE_ID, TURN_ID, ABSOLUTE_ENCODER_ID;
        public final Rotation2d ABSOLUTE_ENCODER_OFFSET;
        public final String MODULE_NAME;
        public final Boolean DRIVE_INVERTED, TURN_INVERTED;

        ModuleSettings(int driveID, Boolean driveInverted, int turnID, Boolean turnInverted, int absoluteEncoderID, Rotation2d absoluteEncoderOffset, String moduleName) {
            this.DRIVE_ID = driveID;
            this.TURN_ID = turnID;
            this.ABSOLUTE_ENCODER_ID = absoluteEncoderID;
            this.ABSOLUTE_ENCODER_OFFSET = absoluteEncoderOffset;
            this.MODULE_NAME = moduleName;
            this.DRIVE_INVERTED = driveInverted;
            this.TURN_INVERTED = turnInverted;
        }
    }

    /**
     * Constants for the MK4SDS swerve modules.
    */
    public static class MK4SDS{
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);
        public static final double DRIVE_MOTOR_GEAR_RATIO = 1 / 6.12;
        public static final double TURNING_MOTOR_GEAR_RATIO = 1 / 12.8;
        public static final double DRIVE_ENCODER_ROT_TO_METER = DRIVE_MOTOR_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_METERS;
        public static final double TURNING_ENCODER_ROT_TO_RAD = TURNING_MOTOR_GEAR_RATIO * 2 * Math.PI;
        public static final double DRIVE_ENCODER_RPM_TO_METER_PER_SEC = DRIVE_ENCODER_ROT_TO_METER / 60;
        public static final double TURNING_ENCODER_RPM_TO_RAD_PER_SEC = TURNING_ENCODER_ROT_TO_RAD / 60;

        // PID constants
        public static final double TURN_MODULE_PID_P = 0.001;
        public static final double TURN_MODULE_PID_I = 0.000001;
        public static final double TURN_MODULE_PID_D = 0.00025;
        public static final double DRIVE_MODULE_PID_P = 0.45;
        public static final double DRIVE_MODULE_PID_I = 0.0;
        public static final double DRIVE_MODULE_PID_D = 0.0;
    }

    /**
     * Constants for the kinematics of the swerve modules.
     */
    public static class Kinematics {
        public static final double TRACK_WIDTH_X = Units.inchesToMeters(21.25); // Distance between RIGHT and LEFT wheel centers
        public static final double TRACK_WIDTH_Y = Units.inchesToMeters(21.25); // Distance between FRONT and BACK wheel centers

        public static final Translation2d[] MODULE_TRANSLATIONS = {
            new Translation2d(-TRACK_WIDTH_X / 2, TRACK_WIDTH_Y / 2), // Front right
            new Translation2d(-TRACK_WIDTH_X / 2, -TRACK_WIDTH_Y / 2), // Back right
            new Translation2d(TRACK_WIDTH_X / 2, TRACK_WIDTH_Y / 2), // Front left
            new Translation2d(TRACK_WIDTH_X / 2, -TRACK_WIDTH_Y / 2), // Back left
        };
        
        /* Junk code for testing
        public static final Translation2d[] MODULE_TRANSLATIONS = {
            new Translation2d(trackWidthX / 2, trackWidthY / 2), //fl
            new Translation2d(-trackWidthX / 2, trackWidthY / 2), //fr
            new Translation2d(trackWidthX / 2, -trackWidthY / 2), //bl
            new Translation2d(-trackWidthX / 2, -trackWidthY / 2), //br
        };

        public static final Translation2d[] MODULE_TRANSLATIONS = {
            // FrontLeft
            new Translation2d(-trackWidthX / 2.0, trackWidthY / 2.0),
            // FrontRight
            new Translation2d(trackWidthX / 2.0, trackWidthY / 2.0),
            // BackLeft
            new Translation2d(-trackWidthX / 2.0, -trackWidthY / 2.0),
            // BackRight
            new Translation2d(trackWidthX / 2.0, -trackWidthY / 2.0)
        };
        */

        // Kinematics for the swerve modules created via the module translations.
        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(MODULE_TRANSLATIONS);

        // Maximum speeds of the robot. Used for unit conversion and controller input scaling.
        public static final double MAX_LINEAR_VELOCITY = 1.0;
        public static final double MAX_ANGULAR_VELOCITY = 1.0;
    }

    public static class Input{

        public static final double DRIVER_DEADBAND = 0.05;
        public static final int DRIVER_PORT = 1;

        public static final double OPERATOR_DEADBAND = 0.05;
        public static final int OPERATOR_PORT = 2;

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

}
