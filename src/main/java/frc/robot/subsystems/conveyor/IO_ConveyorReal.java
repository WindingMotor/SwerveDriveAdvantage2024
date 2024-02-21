// Written by WindingMotor, 2024, Crescendo

package frc.robot.subsystems.conveyor;
import com.revrobotics.CANSparkMax;
import frc.robot.Constants;
import frc.robot.util.Builder;
import frc.robot.util.IRBeamBreak;

public class IO_ConveyorReal implements IO_ConveyorBase{

    private CANSparkMax intakeMotor;       
    private CANSparkMax indexerMotor;  
    
    private IRBeamBreak intakeSensor;

    private IRBeamBreak indexerInitalSensor;
    private IRBeamBreak indexerFinalSensor;
    private boolean shooterFlag = false;
    
    public IO_ConveyorReal(){
        intakeMotor = Builder.createNeo(Constants.Maestro.INTAKE_MOTOR_ID, false, 60);
        indexerMotor = Builder.createNeo(Constants.Maestro.INDEXER_MOTOR_ID, false, 45);

        intakeSensor = new IRBeamBreak(Constants.Maestro.INTAKE_SENSOR_ID);

        indexerInitalSensor = new IRBeamBreak(Constants.Maestro.INDEXER_INITAL_SENSOR_ID);
        indexerFinalSensor = new IRBeamBreak(Constants.Maestro.INDEXER_FINAL_SENSOR_ID);
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

        inputs.intakeSensorState = intakeSensor.getState();
        inputs.indexerInitalSensorState = indexerInitalSensor.getState();
        inputs.indexerFinalSensorState = indexerFinalSensor.getState();
        if(!inputs.indexerFinalSensorState){shooterFlag = true;}
    }

    /**
     * Updates the PID controller with the new setpoint position.
     * @param  setpointPosition  The new setpoint position for the PID controller
    */
    @Override
    public void update(double intakeSpeed, double indexerSpeed){
        intakeMotor.set(intakeSpeed);  
        indexerMotor.set(indexerSpeed);
    }

    /**
     * Stops the arm motors.
    */
    @Override
    public void stop(){
        update(0.0,0.0);
    }

    public boolean getShooterFlag(){
        return shooterFlag;
    }

}
