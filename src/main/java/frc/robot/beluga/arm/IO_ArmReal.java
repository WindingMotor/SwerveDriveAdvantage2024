
package frc.robot.beluga.arm;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
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
        inputs.motorOnePositionDegrees = convertToDegrees(motorOneEncoder.getPosition());
        inputs.motorTwoPositionDegrees = convertToDegrees(motorTwoEncoder.getPosition());
        inputs.setpointPosition = setpointPosition;
        inputs.isAtSetpoint = Math.abs(motorOneEncoder.getPosition() - setpointPosition) < 0.1;

        Mechanism2d mechanism = new Mechanism2d(3, 3);
        Logger.recordOutput("armMechanism", mechanism);

    }

    /**
     * Converts the input value to degrees and applies the arm offset.
     * @param  input	The input value to be converted
     * @return         The converted value in degrees with arm offset applied.
    */
    private double convertToDegrees(double input){
        return motorOneEncoder.getPosition() * 360 - Constants.Beluga.ARM_OFFSET_DEGREES;
    }

    /**
     * Updates the PID controller with the new setpoint position.
     * @param  setpointPosition  The new setpoint position for the PID controller
    */
    @Override
    public void updatePID(double setpointPosition){
        this.setpointPosition = setpointPosition;
        motorOne.set(
            armPID.calculate(convertToDegrees(motorOneEncoder.getPosition()), setpointPosition)
        );
    }

    /**
     * Stops the arm motors.
    */
    @Override
    public void stop(){
        updatePID(0.0);
    }

}
