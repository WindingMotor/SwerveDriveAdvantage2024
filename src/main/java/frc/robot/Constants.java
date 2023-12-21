package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {

    // Current robot mode.
    public static final RobotMode currentMode = RobotMode.REAL;
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
        FRONTLEFT(1,true, 2,true, 0, new Rotation2d(0.0), "FrontLeft"),
        FRONTRIGHT(3,true, 4,true, 1, new Rotation2d(6.11), "FrontRight"),
        BACKLEFT(7,true, 8,true, 3, new Rotation2d(0.43), "BackLeft"),
        BACKRIGHT(5,true, 6,true, 2, new Rotation2d(4.53), "BackRight");

        public final int driveID, turnID, absoluteEncoderID;
        public final Rotation2d absoluteEncoderOffset;
        public final String moduleName;
        public final Boolean driveInverted, turnInverted;

        ModuleSettings(int driveID, Boolean driveInverted, int turnID,  Boolean turnInverted, int absoluteEncoderID, Rotation2d absoluteEncoderOffset, String moduleName) {
            this.driveID = driveID;
            this.turnID = turnID;
            this.absoluteEncoderID = absoluteEncoderID;
            this.absoluteEncoderOffset = absoluteEncoderOffset;
            this.moduleName = moduleName;
            this.driveInverted = driveInverted;
            this.turnInverted = turnInverted;
        }
      
    }

    public static class MK4SDS{
        
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 6.12 ; // old value 1 / 5.8462
        public static final double kTurningMotorGearRatio = 1 / 12.8; // old value 1 / 18.0
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

        // PIDs
        public static final double TURN_MODULE_PID_P = 0.001; // 0.5
        public static final double TURN_MODULE_PID_I = 0.000001; // 0.0
        public static final double TURN_MODULE_PID_D = 0.00025; // 0.0

        public static final double DRIVE_MODULE_PID_P = 0.45;
        public static final double DRIVE_MODULE_PID_I = 0.0;
        public static final double DRIVE_MODULE_PID_D = 0.0;
    }

    public static class Kinematics{
        public static final double trackWidthX = Units.inchesToMeters(21.25);; // Distance between RIGHT and LEFT wheel centers
        public static final double trackWidthY = Units.inchesToMeters(21.25);; // Distance between FRONT and BACK wheel centers
        
        
        public static final Translation2d[] MODULE_TRANSLATIONS = {
            new Translation2d(-trackWidthX / 2, trackWidthY / 2), //fr
            new Translation2d(-trackWidthX / 2, -trackWidthY / 2), //br
            new Translation2d(trackWidthX / 2, trackWidthY / 2), //fl
            new Translation2d(trackWidthX / 2, -trackWidthY / 2), //bl
        };
        
        /*
        
        public static final Translation2d[] MODULE_TRANSLATIONS = {
            new Translation2d(trackWidthX / 2, trackWidthY / 2), //fl
            new Translation2d(-trackWidthX / 2, trackWidthY / 2), //fr
            new Translation2d(trackWidthX / 2, -trackWidthY / 2), //bl
            new Translation2d(-trackWidthX / 2, -trackWidthY / 2), //br
        };
         * 
         * 
         */

        /* 

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

        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(MODULE_TRANSLATIONS);

        // Used for conversion to M/S
        public static final double MAX_LINEAR_VELOCITY = 1.0;
        public static final double MAX_ANGULAR_VELOCITY = 1.0;

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
