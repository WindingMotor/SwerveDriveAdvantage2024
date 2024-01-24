
package frc.robot.beluga.shooter;
import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.wmlib2j.util.Builder;

/**
 * Represents a real implementation of the shooter.
*/
public class IO_ShooterReal implements IO_ShooterBase {

    private CANSparkMax motorOne;       
    private CANSparkMax motorTwo;         
    
    private RelativeEncoder motorOneEncoder;
    private RelativeEncoder motorTwoEncoder;

    private SparkPIDController leftShooterPID;
    private SparkPIDController rightShooterPID;

    private DigitalInput backLimitSwitch;

    private double setpointRPM;

    public IO_ShooterReal(){
        motorOne = Builder.createNeo(Constants.Beluga.SHOOTER_MOTOR_LEFT_ID, Constants.Beluga.SHOOTER_MOTOR_LEFT_INVERTED, 30);
        motorTwo = Builder.createNeo(Constants.Beluga.SHOOTER_MOTOR_RIGHT_ID, Constants.Beluga.SHOOTER_MOTOR_RIGHT_INVERTED, 30);

        Builder.configureIdleMode(motorOne, false);
        Builder.configureIdleMode(motorTwo, false);

        leftShooterPID = motorOne.getPIDController();
        rightShooterPID = motorTwo.getPIDController();

        Builder.configurePIDController(leftShooterPID, false, new PIDConstants(0.1));
        Builder.configurePIDController(rightShooterPID, false, new PIDConstants(0.1));

        motorOneEncoder = motorOne.getEncoder();
        motorTwoEncoder = motorTwo.getEncoder();

        backLimitSwitch = new DigitalInput(Constants.Beluga.SHOOTER_BACK_LIMIT_SWITCH);
    }

    /**
     * Updates the inputs with the current values.
     * @param inputs The inputs to update.
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
     * @param  setpointRPM    The desired setpoint in RPM.
    */
    @Override
    public void updatePID(double setpointRPM){
        this.setpointRPM = setpointRPM;
        leftShooterPID.setReference(setpointRPM, CANSparkMax.ControlType.kVelocity);
        rightShooterPID.setReference(setpointRPM, CANSparkMax.ControlType.kVelocity);
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
