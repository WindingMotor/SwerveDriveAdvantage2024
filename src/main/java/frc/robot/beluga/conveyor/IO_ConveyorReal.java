
package frc.robot.beluga.conveyor;
import com.revrobotics.*;
import frc.robot.Constants;
import frc.robot.wmlib2j.util.Builder;
import frc.robot.wmlib2j.util.IRBeamBreak;

public class IO_ConveyorReal implements IO_ConveyorBase{

    private CANSparkMax intakeMotor;       
    private CANSparkMax indexerMotor;  
    
    private IRBeamBreak intakeInitalSensor;
    private IRBeamBreak intakeFinalSensor;

    private IRBeamBreak idexerInitalSensor;
    private IRBeamBreak indexerFinalSensor;
    
    public IO_ConveyorReal(){
        intakeMotor = Builder.createNeo(Constants.Beluga.INTAKE_MOTOR_ID, false, 35);
        indexerMotor = Builder.createNeo(Constants.Beluga.INDEXER_MOTOR_ID, false, 15);

        intakeInitalSensor = new IRBeamBreak(3);
        intakeFinalSensor = new IRBeamBreak(2);

        idexerInitalSensor = new IRBeamBreak(1);
        indexerFinalSensor = new IRBeamBreak(0);
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

        inputs.intakeInitalSensorState = intakeInitalSensor.getState();
        inputs.intakeFinalSensorState = intakeFinalSensor.getState();
        inputs.indexerInitalSensorState = idexerInitalSensor.getState();
        inputs.indexerFinalSensorState = indexerFinalSensor.getState();
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
