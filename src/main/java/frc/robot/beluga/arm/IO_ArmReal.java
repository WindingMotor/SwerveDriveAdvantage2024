
package frc.robot.beluga.arm;
import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.*;
import com.revrobotics.CANSparkBase.ControlType;
import frc.robot.wmlib2j.util.Builder;

public class IO_ArmReal implements IO_ArmBase{

    private CANSparkMax motorOne;       
    private CANSparkMax motorTwo;         
    
    private RelativeEncoder motorOneEncoder;
    private RelativeEncoder motorTwoEncoder;

    private SparkPIDController motorOnePID;
    private SparkPIDController motorTwoPID;

    private double setpointPosition;

    public IO_ArmReal(){
        motorOne = Builder.createNeo(21, false, 30);
        motorTwo = Builder.setNeoFollower(motorOne, Builder.createNeo(22, false, 30));

        Builder.configureIdleMode(motorOne, true);
        Builder.configureIdleMode(motorTwo, true);

        motorOneEncoder = Builder.createEncoder(motorOne, 0.0105, 0.0105);
        motorTwoEncoder = Builder.createEncoder(motorTwo, 0.0105, 0.0105);

        motorOnePID = motorOne.getPIDController();
        motorTwoPID = motorTwo.getPIDController();

        Builder.configurePIDController(motorOnePID, false, new PIDConstants(0.1));
        Builder.configurePIDController(motorTwoPID, false, new PIDConstants(0.1));
        
    }

    /**
     * Updates the inputs with the current values.
     * @param inputs The inputs to update.
    */
    @Override
    public void updateInputs(ArmInputs inputs){
        inputs.motorOnePosition = motorOneEncoder.getPosition();
        inputs.motorTwoPosition = motorTwoEncoder.getPosition();
        inputs.setpointPosition = setpointPosition;
        inputs.isAtSetpoint = Math.abs(motorOneEncoder.getPosition() - setpointPosition) < 0.1;
    }

    @Override
    public void updatePID(double setpointPosition){
        this.setpointPosition = setpointPosition;
        motorOnePID.setReference(setpointPosition, ControlType.kPosition);
        motorTwoPID.setReference(setpointPosition, ControlType.kPosition);
    }

    @Override
    public void stop(){
        updatePID(0.0);
    }

}
