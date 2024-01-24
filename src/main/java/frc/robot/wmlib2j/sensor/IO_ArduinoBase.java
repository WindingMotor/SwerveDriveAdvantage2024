// Written by WindingMotor as part of the wmlib2j library.

package frc.robot.wmlib2j.sensor;
import org.littletonrobotics.junction.AutoLog;

/**
 * Base interface for using the gyroscope, abstracts to the real and simulation classes.
 * Provides methods for updating inputs.
*/
public interface IO_ArduinoBase{

    /**
     * Represents the inputs for a gyroscope.
     * It contains fields for yaw, pitch, and roll positions.
    */
    @AutoLog
    public static class ArduinoInputs{
        public boolean connected = false;
        public double rawPWMOutput = 0.0;
        public double rawPWMInput = 0.0;
        public boolean triggerOne = false;
        public boolean triggerTwo = false;
        public boolean triggerThree = false;
    }

    /**
     * Updates the inputs with the current values.
     * @param inputs The inputs to update.
    */
    void updateInputs(ArduinoInputs inputs);

}