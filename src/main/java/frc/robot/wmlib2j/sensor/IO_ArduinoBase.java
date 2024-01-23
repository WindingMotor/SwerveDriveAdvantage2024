// Written by WindingMotor as part of the wmlib2j library.

package frc.robot.wmlib2j.sensor;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Base interface for using the gyroscope, abstracts to the real and simulation classes.
 * Provides methods for updating inputs.
*/
public interface IO_ArduinoBase{

    /**
     * Represents the inputs for a gyroscope.
     * It contains fields for yaw, pitch, and roll positions.
    */
    class ArduinoInputs implements LoggableInputs{
        public boolean connected = false;
        public double rawPWMOutput = 0.0;
        public double rawPWMInput = 0.0;
        public boolean triggerOne = false;
        public boolean triggerTwo = false;
        public boolean triggerThree = false;
        

        @Override
        /**
         * Converts the current state of the gyroscope inputs into a loggable format.
         * @param table The log table to write the inputs to.
        */
        public void toLog(LogTable table){
            table.put("connected", connected);
            table.put("rawPWMOutput", rawPWMOutput);
            table.put("rawPWMInput", rawPWMInput);
            table.put("triggerOne", triggerOne);
            table.put("triggerTwo", triggerTwo);
            table.put("triggerThree", triggerThree);
        }

        @Override
        /**
         * Reads the state of the gyroscope inputs from a log table.
         * @param table The log table to read the inputs from.
        */
        public void fromLog(LogTable table){
            connected = table.get("connected", connected);
            rawPWMOutput = table.get("rawPWMOutput", rawPWMOutput);
            rawPWMInput = table.get("rawPWMInput", rawPWMInput);
            triggerOne = table.get("triggerOne", triggerOne);
            triggerTwo = table.get("triggerTwo", triggerTwo);
            triggerThree = table.get("triggerThree", triggerThree);
        }
    }

    /**
     * Updates the inputs with the current values.
     * @param inputs The inputs to update.
    */
    void updateInputs(ArduinoInputs inputs);

}