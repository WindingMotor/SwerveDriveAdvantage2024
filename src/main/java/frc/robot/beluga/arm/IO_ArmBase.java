package frc.robot.beluga.arm;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface IO_ArmBase 
{
    
    class ArmInputs implements LoggableInputs
    {

        public double motorOnePosition = 0.0;
        public double motorTwoPosition = 0.0;
        
        public double motorOneCurrent = 0.0;
        public double motorTwoCurrent = 0.0;

        public double setpointPosition = 0.0;
        public boolean isAtSetpoint = false;
        
        @Override
        /**
         * Converts the current state of the gyroscope inputs into a loggable format.
         * @param table The log table to write the inputs to.
        */
        public void toLog(LogTable table){
            table.put("motorOnePosition", motorOnePosition);
            table.put("motorTwoPosition", motorTwoPosition);
            table.put("motorOneCurrent", motorOneCurrent);
            table.put("motorTwoCurrent", motorTwoCurrent);
            table.put("isAtSetpoint", isAtSetpoint);
            table.put("setpointPosition", setpointPosition);
        }

        @Override
        /**
         * Reads the state of the gyroscope inputs from a log table.
         * @param table The log table to read the inputs from.
        */
        public void fromLog(LogTable table){
            motorOnePosition = table.get("motorOnePosition", motorOnePosition);
            motorTwoPosition = table.get("motorTwoPosition", motorTwoPosition);
            motorOneCurrent = table.get("motorOneCurrent", motorOneCurrent);
            motorTwoCurrent = table.get("motorTwoCurrent", motorTwoCurrent);
            setpointPosition = table.get("setpointPosition", setpointPosition);
            isAtSetpoint = table.get("isAtSetpoint", isAtSetpoint);
        } 
    }
}
    


