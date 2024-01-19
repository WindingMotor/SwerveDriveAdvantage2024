
package frc.robot.beluga.shooter;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Base interface for the shooter, abstracts to the real and simulation classes.
 * Provides methods for updating inputs.
*/
public interface IO_ShooterBase{

    /**
     * Represents the inputs for a gyroscope.
     * It contains fields for yaw, pitch, and roll positions.
    */
    class ShooterInputs implements LoggableInputs{
        //RPM = Rotation/Minute
        public double leftMotorRPM = 0.0;
        public double rightMotorRPM = 0.0;
        public double setpointRPM = 0.0;
        public boolean isUpToSpeed = false;
        public boolean backLimitSwitchStatus = false;


        @Override
        /**
         * Converts the current state of the gyroscope inputs into a loggable format.
         * @param table The log table to write the inputs to.
        */
        public void toLog(LogTable table){
            table.put("leftMotorRPM", leftMotorRPM);
            table.put("rightMotorRPM", rightMotorRPM);
            table.put("setpointRPM", setpointRPM);
            table.put("isUpToSpeed", isUpToSpeed);
            table.put("backLimitSwitchStatus", backLimitSwitchStatus);
        }

        @Override
        /**
         * Reads the state of the gyroscope inputs from a log table.
         * @param table The log table to read the inputs from.
        */
        public void fromLog(LogTable table){
            leftMotorRPM = table.get("leftMotorRPM", leftMotorRPM);
            rightMotorRPM = table.get("rightMotorRPM", rightMotorRPM);
            setpointRPM = table.get("setpointRPM", setpointRPM);
            isUpToSpeed = table.get("isUpToSpeed", isUpToSpeed);
            backLimitSwitchStatus = table.get("backLimitSwitchStatus", backLimitSwitchStatus);
        }
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
