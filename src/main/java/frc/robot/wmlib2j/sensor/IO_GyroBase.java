
package frc.robot.wmlib2j.sensor;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Base interface for using the gyroscope, abstracts to the real and simulation classes.
 * Provides methods for updating inputs.
*/
public interface IO_GyroBase{

    /**
     * Represents the inputs for a gyroscope.
     * It contains fields for yaw, pitch, and roll positions.
    */
    class GyroIOInputs implements LoggableInputs{
        public boolean connected = false;
        public Rotation2d yawPosition = new Rotation2d();
        public double yawPositionRadians = 0.0;
        public double yawPositionDegrees = 0.0;
        public Rotation2d pitchPosition = new Rotation2d();
        public Rotation2d rollPosition = new Rotation2d();

        @Override
        /**
         * Converts the current state of the gyroscope inputs into a loggable format.
         * @param table The log table to write the inputs to.
        */
        public void toLog(LogTable table){
            table.put("connected", connected);
            table.put("yawPosition", yawPosition);
            table.put("yawPositionRadians", yawPositionRadians);
            table.put("yawPositionDegrees", yawPositionDegrees);
            table.put("pitchPosition", pitchPosition);
            table.put("rollPosition", rollPosition);
        }

        @Override
        /**
         * Reads the state of the gyroscope inputs from a log table.
         * @param table The log table to read the inputs from.
        */
        public void fromLog(LogTable table){
            connected = table.get("connected", connected);
            yawPosition = table.get("yawPosition", yawPosition);
            yawPositionRadians = table.get("yawPositionRadians", yawPositionRadians);
            yawPositionDegrees = table.get("yawPositionDegrees", yawPositionDegrees);
            pitchPosition = table.get("encoderVelocity", pitchPosition);
            rollPosition = table.get("encoderVelocity", rollPosition);
        }
    }

    /**
     * Updates the inputs with the current values.
     * @param inputs The inputs to update.
    */
    void updateInputs(GyroIOInputs inputs);

}