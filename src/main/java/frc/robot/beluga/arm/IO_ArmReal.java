
package frc.robot.beluga.arm;
import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.*;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.wmlib2j.util.Builder;

public class IO_ArmReal implements IO_ArmBase{

    private CANSparkMax motorOne;       
    private CANSparkMax motorTwo;         
    
    private RelativeEncoder motorOneEncoder;
    private RelativeEncoder motorTwoEncoder;

    private PIDController armPID;

    private double setpointPosition;

    public IO_ArmReal(){
        motorOne = Builder.createNeo(Constants.Beluga.ARM_MOTOR_LEAD_ID, Constants.Beluga.ARM_MOTOR_LEAD_INVERTED, 30);
        motorTwo = Builder.setNeoFollower(motorOne, Builder.createNeo(Constants.Beluga.ARM_MOTOR_FOLLOWER_ID, Constants.Beluga.ARM_MOTOR_FOLLOWER_INVERTED, 30));

        Builder.configureIdleMode(motorOne, true);
        Builder.configureIdleMode(motorTwo, true);

        motorOneEncoder = Builder.createEncoder(motorOne, 0.0105, 0.0105);
        motorTwoEncoder = Builder.createEncoder(motorTwo, 0.0105, 0.0105);

        armPID = new PIDController(Constants.Beluga.ARM_MOTORS_P, Constants.Beluga.ARM_MOTORS_I, Constants.Beluga.ARM_MOTORS_D);


        //Builder.configurePIDController(motorOnePID, false, new PIDConstants(Constants.Beluga.ARM_MOTORS_P, Constants.Beluga.ARM_MOTORS_I, Constants.Beluga.ARM_MOTORS_D));
        //Builder.configurePIDController(motorTwoPID, false, new PIDConstants(Constants.Beluga.ARM_MOTORS_P, Constants.Beluga.ARM_MOTORS_I, Constants.Beluga.ARM_MOTORS_D));
        
        Builder.configureIdleMode(motorOne, false);
        Builder.configureIdleMode(motorTwo, false);
    }

    /**
     * Updates the inputs with the current values.
     * @param inputs The inputs to update.
    */
    @Override
    public void updateInputs(ArmInputs inputs){
        inputs.motorOnePosition = getMotorOnePosition();
        inputs.motorTwoPosition = motorTwoEncoder.getPosition() * 360 - 19.5;
        inputs.setpointPosition = setpointPosition;
        inputs.isAtSetpoint = Math.abs(motorOneEncoder.getPosition() - setpointPosition) < 0.1;
    }

    private double getMotorOnePosition(){
        return motorOneEncoder.getPosition() * 360 - 19.5;
    }

    @Override
    public void updatePID(double setpointPosition){
        this.setpointPosition = setpointPosition;
        motorOne.set(
            armPID.calculate(getMotorOnePosition(), setpointPosition)
        );
        //motorOnePID.setReference(setpointPosition, ControlType.kPosition);
        //motorTwoPID.setReference(setpointPosition, ControlType.kPosition);
    }

    @Override
    public void stop(){
        updatePID(0.0);
    }

}
