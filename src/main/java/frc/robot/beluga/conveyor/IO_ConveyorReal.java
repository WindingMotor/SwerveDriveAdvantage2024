
package frc.robot.beluga.conveyor;
import com.revrobotics.*;
import frc.robot.Constants;
import frc.robot.wmlib2j.util.Builder;
import frc.robot.wmlib2j.util.GP2Y0A21YK0F;

public class IO_ConveyorReal implements IO_ConveyorBase{

    private CANSparkMax intakeMotor;       
    private CANSparkMax indexerMotor;  
    
    private GP2Y0A21YK0F intakeInitalSensor;
    private GP2Y0A21YK0F intakeFinalSensorState;

    private GP2Y0A21YK0F idexerInitalSensor;
    private GP2Y0A21YK0F indexerFinalSensorState;
    
    public IO_ConveyorReal(){
        intakeMotor = Builder.createNeo(Constants.Beluga.INTAKE_MOTOR_ID, false, 15);
        indexerMotor = Builder.createNeo(Constants.Beluga.INDEXER_MOTOR_ID, false, 15);
    }

    /**
     * Updates the inputs with the current values.
     * @param inputs The inputs to update.
    */
    @Override
    public void updateInputs(ConveyorInputs inputs){
        inputs.intakeMotorSpeed = intakeMotor.getEncoder().getVelocity();
        inputs.intakeMotorCurrent = intakeMotor.getOutputCurrent();
        inputs.indexerMotorSpeed = indexerMotor.getEncoder().getVelocity();
        inputs.indexerMotorCurrent = indexerMotor.getOutputCurrent();

        inputs.intakeInitalSensorState = intakeInitalSensor.isWithinRange();
        inputs.intakeFinalSensorState = intakeFinalSensorState.isWithinRange();
        inputs.indexerInitalSensorState = idexerInitalSensor.isWithinRange();
        inputs.indexerFinalSensorState = indexerFinalSensorState.isWithinRange();
    }

    /**
     * Updates the PID controller with the new setpoint position.
     * @param  setpointPosition  The new setpoint position for the PID controller
    */
    @Override
    public void update(double intakeSpeed, double indexerSpeed){
     //   intakeMotor.set(indexerSpeed);  
        indexerMotor.set(indexerSpeed);
    }

    /**
     * Stops the arm motors.
    */
    @Override
    public void stop(){
        update(0.0,0.0);
    }

}
