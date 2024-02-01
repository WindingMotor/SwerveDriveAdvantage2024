package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

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

    public static class Vision {

        public static final double LATENCY_THRESHOLD_MILI_SEC = 30.0;

        public enum Camera{
        LEFT_CAMERA("OV9281_02", "leftCamera", new Transform3d(
            new Translation3d(11.0, -13.5, 12.0), 
            new Rotation3d(0,0,0)) // Camera mounted 10.25in forward, -12.5in left, 12.0in up facing forward.
        ),
        RIGHT_CAMERA("OV9281_01", "rightCamera", new Transform3d(
            new Translation3d(11.5, 9.5, 8.5), 
            new Rotation3d(0,0,0)) // Camera mounted 8.5in forward, 8.5in right, 9.0in up facing forward.
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

    public static class Beluga{

        // Arm
        public static final int ARM_MOTOR_LEAD_ID = 9;
        public static final int ARM_MOTOR_FOLLOWER_ID = 10;

        public static final boolean ARM_MOTOR_LEAD_INVERTED = true;
        public static final boolean ARM_MOTOR_FOLLOWER_INVERTED = true;

        public static final double ARM_MOTORS_P = 0.015;
        public static final double ARM_MOTORS_I = 0.0001;
        public static final double ARM_MOTORS_D = 0.0006;

        public static final double ARM_SLEW_RATE = 25; // Smaller slower

        public static final double ARM_POSITION_TOLERANCE_DEGREES = 0.25;
        public static final double ARM_OFFSET_DEGREES = 19.0;

        // Shooter
        public static final int SHOOTER_MOTOR_LEFT_ID = 12;
        public static final int SHOOTER_MOTOR_RIGHT_ID = 13;

        public static final boolean SHOOTER_MOTOR_LEFT_INVERTED = false;
        public static final boolean SHOOTER_MOTOR_RIGHT_INVERTED = true;

        // 6e-5
        public static final double SHOOTER_MOTORS_P = 0.00065; 
        public static final double SHOOTER_MOTORS_I = 0.0;
        public static final double SHOOTER_MOTORS_D = 0.00005;
        public static final double SHOOTER_MOTORS_IZ = 0.0005;
        public static final double SHOOTER_MOTORS_FF = 0.00018; 

        public static final double SHOOTER_SPEED_TOLERANCE_RPM = 3.0;

        public static final int SHOOTER_BACK_LIMIT_SWITCH = 0;

        // Intake
        public static final int INDEXER_MOTOR_ID = 11;
 
        // Indexer
        public static final int INTAKE_MOTOR_ID = 14;

        // Sensor IDs
        public static final int INTAKE_INITAL_SENSOR_ID = 0;
        public static final int INTAKE_FINAL_SENSOR_ID = 1;

        public static final int INDEXER_INITAL_SENSOR_ID = 2;
        public static final int INDEXER_FINAL_SENSOR_ID = 3;

        public static final int SENSOR_TOLERANCE_MAX_CENTIMETERS = 12;
        public static final int SENSOR_TOLERANCE_MIN_CENTIMETERS = 8;


        /*
            Motor IDs
            1-8 swerve
            9-10 arm
            11 indexer
            12-13 shooter
        */


        
    }

    public static class Arduino{

        public static final double PWM_PORT = 5;

        public enum OutputMapping{
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

    public static class States{

        public enum ShooterMode{
            SPEAKER,
            AMP,
            DYNAMIC
        }

        public enum ShooterState{
            OFF(0, "OFF"), // No motors running
            IDLE(2000, "IDLE"), // Shooter running at lower rpm but ready to spin up
            SPEAKER(3500, "SPEAKER"), // Shooter running at constant rpm for speaker scoring
            AMP(500, "AMP"), // Shooter running at constant rpm for direct amp scoring
            DYNAMIC(-1.0, "DYNAMIC"), // Shooter auto calculating rpm depending on distance to speaker 
            DEMO(3500, "DEMO"); // Shooter running at constant rpm for demo purposes

            public final double rpm;
            public final String name;

            ShooterState(double rpm, String name) {
                this.rpm = rpm;
                this.name = name;
            }
        }

        public enum ArmState{
            OFF(-18, "OFF"), // No motors running
            IDLE(-15, "IDLE"), // Arm sitting on right above hard stops and ready to move
            SPEAKER(55.0, "SPEAKER"), // Arm at position needed for speaker
            AMP(96, "AMP"), // Arm at position needed for direct amp scoring,
            DYNAMIC(-1.0, "DYNAMIC"), // Arm calculating position depending on distance to speaker 
            INTAKE(110.0, "INTAKE"), // Arm at position needed for intake pickup
            DEMO(15, "DEMO"); // Arm at position needed for demo purposes

            public final double position;
            public final String name;

            ArmState(double position, String name) {
                this.position = position;
                this.name = name;
            }
        }

        public enum ConveyorState{
            OFF(0.0, 0.0, "OFF"), // No motors running
            INTAKE(0.9, 0.65, "INTAKE"), // Intake and Indexer running
            INTAKE_REV(0.9, -0.45, "INTAKE REV"), // Intake and Indexer running backwards
            SHOOT(0.0, 0.8, "SHOOT"); // Indexer for shooting

            public final double intakeSpeed, indexerSpeed;
            public final String name;

            ConveyorState(double intakeSpeed, double indexerSpeed, String name) {
                this.intakeSpeed = intakeSpeed;
                this.indexerSpeed = indexerSpeed;
                this.name = name;
            }
        }

    }

    public static class Auto{

        public static final double MAX_VELOCITY_MPS = 1.5;
        public static final double MAX_ACCELERATION_MPS_SQ = 2.0;
        
        public static final double MAX_ANGULAR_VELOCITY_RADS = 360;
        public static final double MAX_ANGULAR_VELOCITY_RADS_SQ = 540;

        public enum ScoringPoses{
            BLU_SPEAKER(new Pose2d(1.5, 5.5, Rotation2d.fromDegrees(180))),
            RED_SPEAKER(new Pose2d(12.0, 5.5, Rotation2d.fromDegrees(180))),

            BLU_AMP(new Pose2d(1.85, 7.75, Rotation2d.fromDegrees(90))),
            RED_AMP(new Pose2d(15.0, 14.75, Rotation2d.fromDegrees(90)));
        
            public final Pose2d pose;
        
            ScoringPoses(Pose2d pose){
                this.pose = pose;
            }
        }
        
        public enum NotePoses{
            BLU_TOP(new Pose2d(2.9, 7.0, Rotation2d.fromDegrees(0))),
            BLU_MIDDLE(new Pose2d(2.9, 5.5, Rotation2d.fromDegrees(0))),
            BLU_BOTTOM(new Pose2d(2.9, 4.1, Rotation2d.fromDegrees(0))),
    
            RED_TOP(new Pose2d(13.67, 7.0, Rotation2d.fromDegrees(0))),
            RED_MIDDLE(new Pose2d(13.67, 5.5, Rotation2d.fromDegrees(0))),
            RED_BOTTOM(new Pose2d(13.67, 4.1, Rotation2d.fromDegrees(0)));
        
            public final Pose2d pose;
        
            NotePoses(Pose2d pose){
                this.pose = pose;
            }
        }
        
    }

}





