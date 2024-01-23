
package frc.robot.beluga.shooter;
import org.littletonrobotics.junction.AutoLog;


/**
 * Base interface for the shooter, abstracts to the real and simulation classes.
 * Provides methods for updating inputs.
*/
public interface IO_ShooterBase{

    /**
     * Represents the inputs for a gyroscope.
     * It contains fields for yaw, pitch, and roll positions.
    */
    @AutoLog
    public static class ShooterInputs{
        //RPM = Rotation/Minute
        public double leftMotorRPM = 0.0;
        public double rightMotorRPM = 0.0;
        public double setpointRPM = 0.0;
        public boolean isUpToSpeed = false;
        public boolean backLimitSwitchStatus = false;
    }

    /**
     * Updates the inputs with the current values.
     * @param inputs The inputs to update.
    */
    void updateInputs(ShooterInputs inputs);

    void updateSetpoint(double setpointRPM);

    void stop();

    boolean isUpToSpeed();

}
