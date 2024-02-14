// Written by WindingMotor, 2024, Crescendo

package frc.robot.subsystems.conveyor;
import org.littletonrobotics.junction.AutoLog;

public interface IO_ConveyorBase{
    
    @AutoLog
    public static class ConveyorInputs{

        public double intakeMotorSpeed = 0.0;
        public double indexerMotorSpeed = 0.0;
        
        public double intakeMotorCurrent = 0.0;
        public double indexerMotorCurrent = 0.0;

        public boolean intakeInitalSensorState = false;
        public boolean intakeFinalSensorState = false;

        public boolean indexerInitalSensorState = false;
        public boolean indexerFinalSensorState = false;
    }

    /**
     * Updates the inputs with the current values.
     * @param inputs The inputs to update.
    */
    void updateInputs(ConveyorInputs inputs);

    void update(double intakeSpeed, double indexerSpeed);
    
    void stop();

}
    


