
package frc.robot.beluga.shooter;
import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.wmlib2j.util.Builder;

/**
 * Represents a real implementation of the shooter.
*/
public class IO_ShooterReal implements IO_ShooterBase {

    private CANSparkFlex motorOne;       
    private CANSparkFlex motorTwo;         
    
    private RelativeEncoder motorOneEncoder;
    private RelativeEncoder motorTwoEncoder;
    
    private SparkPIDController leftPID;

    private SparkPIDController rightPID;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

    private DigitalInput backLimitSwitch;

    private double setpointRPM;

    public IO_ShooterReal(){
        motorOne = Builder.createVortex(Constants.Beluga.SHOOTER_MOTOR_LEFT_ID, Constants.Beluga.SHOOTER_MOTOR_LEFT_INVERTED, 45);
        motorTwo = Builder.createVortex(Constants.Beluga.SHOOTER_MOTOR_RIGHT_ID, Constants.Beluga.SHOOTER_MOTOR_RIGHT_INVERTED, 45);

        Builder.configureIdleMode(motorOne, false);
        Builder.configureIdleMode(motorTwo, false);

        leftPID = motorOne.getPIDController();
        rightPID = motorTwo.getPIDController();

        Builder.configurePIDController(leftPID, false, new PIDConstants(Constants.Beluga.SHOOTER_MOTORS_P, Constants.Beluga.SHOOTER_MOTORS_I, Constants.Beluga.SHOOTER_MOTORS_D), Constants.Beluga.SHOOTER_MOTORS_IZ, Constants.Beluga.SHOOTER_MOTORS_FF);
        
        motorOneEncoder = motorOne.getEncoder();
        motorTwoEncoder = motorTwo.getEncoder();

        backLimitSwitch = new DigitalInput(Constants.Beluga.SHOOTER_BACK_LIMIT_SWITCH);

        // TESTING PID coefficients
        kP = 6e-5; 
        kI = 0;
        kD = 0; 
        kIz = 0; 
        kFF = 0.000015; 
        maxRPM = 5700;
    }

    /**
     * Updates the inputs with the current values.
     * @param inputs The inputs to update
    */
    @Override
    public void updateInputs(ShooterInputs inputs){

        inputs.motorOneRPM = motorOneEncoder.getVelocity();
        inputs.motorTwoRPM = motorTwoEncoder.getVelocity();
        inputs.setpointRPM = setpointRPM;
        inputs.isUpToSpeed = isUpToSpeed();
        inputs.backLimitSwitchStatus = backLimitSwitch.get();
    }
    
    /**
     * Updates the setpoint RPM for the left and right shooter PIDs.
     * @param  setpointRPM    The desired setpoint in RPM
    */
    @Override
    public void updatePID(double setpointRPM){
        this.setpointRPM = setpointRPM;

        leftPID.setReference(setpointRPM, CANSparkFlex.ControlType.kVelocity);
        rightPID.setReference(setpointRPM, CANSparkFlex.ControlType.kVelocity);
    }

    /**
     * Stops the shooter by setting the PID setpoint to 0.
    */
    @Override
    public void stop(){
        updatePID(0.0);
    }
    
    /**
     * Checks if the left and right shooter velocities are up to the setpoint.
     * @return True if both velocities are within the acceptable tolerance, false otherwise
    */
    @Override
    public boolean isUpToSpeed() {
        double leftVelocity = motorOneEncoder.getVelocity();
        double rightVelocity = motorOneEncoder.getVelocity();
    
        boolean isLeftUpToSpeed = leftVelocity >= setpointRPM - Constants.Beluga.SHOOTER_SPEED_TOLERANCE_RPM
                && leftVelocity <= setpointRPM + Constants.Beluga.SHOOTER_SPEED_TOLERANCE_RPM;
    
        boolean isRightUpToSpeed = rightVelocity >= setpointRPM - Constants.Beluga.SHOOTER_SPEED_TOLERANCE_RPM
                && rightVelocity <= setpointRPM + Constants.Beluga.SHOOTER_SPEED_TOLERANCE_RPM;
    
        return isLeftUpToSpeed && isRightUpToSpeed;
    }

}
