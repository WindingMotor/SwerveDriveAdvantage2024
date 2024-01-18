
package frc.robot.beluga.shooter;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

/**
 * Represents a real implementation of the shooter.
*/
public class IO_ShooterReal implements IO_ShooterBase {

    private CANSparkMax leftShooterMotor;       
    private CANSparkMax rightShooterMotor;         
    
    private RelativeEncoder leftShooterEncoder;
    private RelativeEncoder rightShooterEncoder;

    private SparkMaxPIDController leftShooterPID;
    private SparkMaxPIDController rightShooterPID;

    private DigitalInput backLimitSwitch;

    private double setpointRPM;

    public IO_ShooterReal(){
        leftShooterMotor = createMotor(Constants.Beluga.SHOOTER_MOTOR_LEFT_ID, Constants.Beluga.SHOOTER_MOTOR_LEFT_INVERTED, 30);
        rightShooterMotor = createMotor(Constants.Beluga.SHOOTER_MOTOR_RIGHT_ID, Constants.Beluga.SHOOTER_MOTOR_RIGHT_INVERTED, 30);

        leftShooterPID = createPIDController(leftShooterMotor);
        rightShooterPID = createPIDController(rightShooterMotor);

        leftShooterEncoder = leftShooterMotor.getEncoder();
        rightShooterEncoder = rightShooterMotor.getEncoder();

        backLimitSwitch = new DigitalInput(Constants.Beluga.SHOOTER_BACK_LIMIT_SWITCH);
    }

    /**
     * Creates a motor with the given CAN ID and inversion setting.
     * @param id The CAN ID of the motor.
     * @param inverted Whether the direction motor is inverted.
     * @return The created motor.
    */
    private CANSparkMax createMotor(int id, boolean inverted, int currentLimit){
        CANSparkMax motor = new CANSparkMax(id, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kCoast);
        motor.setSmartCurrentLimit(currentLimit);
        motor.setInverted(inverted);
        return motor;
    }

    /**
     * Creates a PID controller for a SparkMax.
     * @param  motor The SparkMax to create the PID controller for.
     * @return The created SparkMaxPIDController.
    */
    private SparkMaxPIDController createPIDController(CANSparkMax motor){
        SparkMaxPIDController pid = motor.getPIDController();
        pid.setP(Constants.Beluga.SHOOTER_MOTORS_P);
        pid.setI(Constants.Beluga.SHOOTER_MOTORS_I);
        pid.setD(Constants.Beluga.SHOOTER_MOTORS_D);
        return pid;
    }

    /**
     * Updates the setpoint RPM for the left and right shooter PIDs.
     * @param  setpointRPM    The desired setpoint in RPM.
    */
    @Override
    public void updateSetpoint(double setpointRPM){
        this.setpointRPM = setpointRPM;
        leftShooterPID.setReference(setpointRPM, CANSparkMax.ControlType.kVelocity);
        rightShooterPID.setReference(setpointRPM, CANSparkMax.ControlType.kVelocity);
    }

    /**
     * Stops the shooter by setting the PID setpoint to 0.
    */
    @Override
    public void stop(){
        setpointRPM = 0.0;
        leftShooterPID.setReference(0, CANSparkMax.ControlType.kVelocity);
        rightShooterPID.setReference(0, CANSparkMax.ControlType.kVelocity);
    }
    
    /**
     * Checks if the left and right shooter velocities are up to the setpoint.
     * @return True if both velocities are within the acceptable tolerance, false otherwise
    */
    @Override
    public boolean isUpToSpeed() {
        double leftVelocity = leftShooterEncoder.getVelocity();
        double rightVelocity = rightShooterEncoder.getVelocity();
    
        boolean isLeftUpToSpeed = leftVelocity >= setpointRPM - Constants.Beluga.SHOOTER_SPEED_TOLERANCE_RPM
                && leftVelocity <= setpointRPM + Constants.Beluga.SHOOTER_SPEED_TOLERANCE_RPM;
    
        boolean isRightUpToSpeed = rightVelocity >= setpointRPM - Constants.Beluga.SHOOTER_SPEED_TOLERANCE_RPM
                && rightVelocity <= setpointRPM + Constants.Beluga.SHOOTER_SPEED_TOLERANCE_RPM;
    
        return isLeftUpToSpeed && isRightUpToSpeed;
    }
    
    /**
     * Updates the inputs with the current values.
     * @param inputs The inputs to update.
    */
    @Override
    public void updateInputs(ShooterInputs inputs){
        inputs.leftMotorRPM = leftShooterEncoder.getVelocity();
        inputs.rightMotorRPM = rightShooterEncoder.getVelocity();
        inputs.setpointRPM = setpointRPM;
        inputs.isUpToSpeed = isUpToSpeed();
        inputs.backLimitSwitchStatus = backLimitSwitch.get();
    }
}
