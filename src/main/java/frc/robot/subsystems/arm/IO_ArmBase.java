// Written by WindingMotor, 2024, Crescendo
package frc.robot.subsystems.arm;
import org.littletonrobotics.junction.AutoLog;

public interface IO_ArmBase {
    
    @AutoLog
    public static class ArmInputs{

        public double armPositionDegrees = 0.0;
        
        public double motorOneCurrent = 0.0;
        public double motorTwoCurrent = 0.0;

        public double setpointPosition = 0.0;
        public boolean isAtSetpoint = false;

        public double pidOutputVolts = 0.0;
        public double ffOutputVolts = 0.0;
        public double pidError = 0.0;
    }

    /**
     * Updates the inputs with the current values.
     * @param inputs The inputs to update.
    */
    void updateInputs(ArmInputs inputs);

    void updatePID(double newSetpoint);
    
    void stop();

    void setAngle(double angle);

}
    


