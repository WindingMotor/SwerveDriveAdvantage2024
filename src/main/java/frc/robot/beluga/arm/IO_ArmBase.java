package frc.robot.beluga.arm;
import org.littletonrobotics.junction.AutoLog;

public interface IO_ArmBase {
    
    @AutoLog
    public static class ArmInputs{

        public double motorOnePositionDegrees = 0.0;
        public double motorTwoPositionDegrees = 0.0;
        
        public double motorOneCurrent = 0.0;
        public double motorTwoCurrent = 0.0;

        public double setpointPosition = 0.0;
        public boolean isAtSetpoint = false;
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
    


