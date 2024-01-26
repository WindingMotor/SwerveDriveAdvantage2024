package frc.robot;
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

        public static final int SHOOTER_MOTOR_LEFT_ID = 17;
        public static final int SHOOTER_MOTOR_RIGHT_ID = 18;

        public static final boolean SHOOTER_MOTOR_LEFT_INVERTED = false;
        public static final boolean SHOOTER_MOTOR_RIGHT_INVERTED = false;

        public static final double SHOOTER_MOTORS_P = 0.01;
        public static final double SHOOTER_MOTORS_I = 0.0;
        public static final double SHOOTER_MOTORS_D = 0.0;

        public static final double SHOOTER_SPEED_TOLERANCE_RPM = 5.0;

        public static final boolean AUTO_SPINUP = false;

        public static final int SHOOTER_BACK_LIMIT_SWITCH = 0;

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

        public static final double MOTOR_ONE_ID = -1;
        public static final double MOTOR_TWO_ID = -1;

        public enum ShooterState{
            OFF(0, "OFF"), // No motors running
            IDLE(0, "IDLE"), // Shooter running at lower rpm but ready to spin up
            SPEAKER(0, "SPEAKER"), // Shooter running at constant rpm for speaker scoring
            AMP(0, "SPEAKER"), // Shooter running at constant rpm for direct amp scoring
            TARGET(-1.0, "TARGET"), // Shooter auto calculating rpm depending on distance to speaker 
            DEMO(2000.0, "DEMO"); // Shooter running at constant rpm for demo purposes

            public final double rpm;
            public final String name;

            ShooterState(double rpm, String name) {
                this.rpm = rpm;
                this.name = name;
            }

        }

        public enum ArmState{
            OFF(0, "OFF"), // No motors running
            IDLE(0, "IDLE"), // Arm sitting on right above hard stops and ready to move
            SPEAKER(0, "SPEAKER"), // Arm at position needed for speaker
            AMP(0, "AMP"), // Arm at position needed for direct amp scoring
            TARGET(-1.0, "TARGET"), // Arm calculating position depending on distance to speaker 
            DEMO(30.0, "DEMO"); // Arm at position needed for demo purposes

            public final double position;
            public final String name;

            ArmState(double position, String name) {
                this.position = position;
                this.name = name;
            }

        }

        public enum SuperStructureState{
            OFF(ArmState.OFF, ShooterState.OFF, "Robot has disabled the subsystems"),
            IDLE(ArmState.OFF, ShooterState.OFF, "Robot is idle"), 
            SPEAKER(ArmState.OFF, ShooterState.OFF, "Robot is in speaker mode"),
            AMP(ArmState.OFF, ShooterState.OFF, "Robot is in amp mode"), 
            TARGET(ArmState.OFF, ShooterState.OFF, "Robot is in target mode"), 
            DEMO(ArmState.OFF, ShooterState.OFF, "Robot is in demo mode");

            public final ArmState armState;
            public final ShooterState shooterState;
            public final String name;

            SuperStructureState(ArmState armState, ShooterState shooterState, String name) {
                this.armState = armState;
                this.shooterState = shooterState;
                this.name = name;
            }

        }


    }

    public static class Arm{

        public static final double MOTOR_ONE_ID = -1;
        public static final double MOTOR_TWO_ID = -1;
    
    }

}





