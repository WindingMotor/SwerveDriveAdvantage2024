
package frc.robot.beluga.shooter;
import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.wmlib2j.util.Builder;

/**
 * Represents a real implementation of the shooter.
*/
public class IO_ShooterReal implements IO_ShooterBase {

    private CANSparkFlex motorBottom;       
    private CANSparkFlex motorTop;         
    
    private RelativeEncoder motorOneEncoder;
    private RelativeEncoder motorTwoEncoder;
    
    private SparkPIDController leftPID;

    private SparkPIDController rightPID;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

    private DigitalInput backLimitSwitch;

    private double setpointRPM;

    public IO_ShooterReal(){
        motorBottom = Builder.createVortex(Constants.Beluga.SHOOTER_MOTOR_BOTTOM_ID, Constants.Beluga.SHOOTER_MOTOR_BOTTOM_INVERTED, 45);
        motorTop = Builder.createVortex(Constants.Beluga.SHOOTER_MOTOR_TOP_ID, Constants.Beluga.SHOOTER_MOTOR_TOP_INVERTED, 45);

        Builder.configureIdleMode(motorBottom, false);
        Builder.configureIdleMode(motorTop, false);

        leftPID = motorBottom.getPIDController();
        rightPID = motorTop.getPIDController();

        Builder.configurePIDController(leftPID, false, new PIDConstants(Constants.Beluga.SHOOTER_MOTORS_P, Constants.Beluga.SHOOTER_MOTORS_I, Constants.Beluga.SHOOTER_MOTORS_D), Constants.Beluga.SHOOTER_MOTORS_IZ, Constants.Beluga.SHOOTER_MOTORS_FF);
        Builder.configurePIDController(rightPID, false, new PIDConstants(Constants.Beluga.SHOOTER_MOTORS_P, Constants.Beluga.SHOOTER_MOTORS_I, Constants.Beluga.SHOOTER_MOTORS_D), Constants.Beluga.SHOOTER_MOTORS_IZ, Constants.Beluga.SHOOTER_MOTORS_FF);
        
        motorOneEncoder = motorBottom.getEncoder();
        motorTwoEncoder = motorTop.getEncoder();

        backLimitSwitch = new DigitalInput(Constants.Beluga.SHOOTER_BACK_LIMIT_SWITCH);

        SmartDashboard.putNumber("shooterSpeed", 0.0);
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

        double shooterSpeed = SmartDashboard.getNumber("shooterSpeed",  0);


        this.setpointRPM = setpointRPM;
        leftPID.setReference(shooterSpeed, CANSparkFlex.ControlType.kVelocity);
        rightPID.setReference(shooterSpeed /*+ setpointRPM * 0.12*/, CANSparkFlex.ControlType.kVelocity);
    }

    /**
     * Stops the shooter by setting the PID setpoint to 0.
    */
    @Override
    public void stop(){
        updatePID(0.0);
    }

    @Override
    public void setRPM(double rpm){
        updatePID(rpm);
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

    @Override
    public void invertMotors(boolean inverted){
       // motorBottom.setInverted(inverted);
        motorTop.setInverted(inverted);
    }

}
