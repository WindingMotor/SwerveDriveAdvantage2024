// Written by WindingMotor, 2024, Crescendo

package frc.robot.subsystems.shooter;
import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.Builder;

/**
 * Represents a real implementation of the shooter.
*/
public class IO_ShooterReal implements IO_ShooterBase {

    private CANSparkFlex bottomMotor;       
    private CANSparkFlex topMotor;         
    
    private RelativeEncoder bottomMotorEncoder;
    private RelativeEncoder topMotorEncoder;
    
    private SparkPIDController bottomPID;

    private SparkPIDController topPID;

    private double setpointRPM;

    public IO_ShooterReal(){

        setpointRPM = 0;

        bottomMotor = Builder.createVortex(Constants.Robot.SHOOTER_MOTOR_BOTTOM_ID, Constants.Robot.SHOOTER_MOTOR_BOTTOM_INVERTED, 45);
        topMotor = Builder.createVortex(Constants.Robot.SHOOTER_MOTOR_TOP_ID, Constants.Robot.SHOOTER_MOTOR_TOP_INVERTED, 45);

        Builder.configureIdleMode(bottomMotor, false);
        Builder.configureIdleMode(topMotor, false);

        bottomPID = bottomMotor.getPIDController();
        topPID = topMotor.getPIDController();

        Builder.configurePIDController(bottomPID, false, new PIDConstants(Constants.Robot.SHOOTER_MOTORS_P, Constants.Robot.SHOOTER_MOTORS_I, Constants.Robot.SHOOTER_MOTORS_D), Constants.Robot.SHOOTER_MOTORS_IZ, Constants.Robot.SHOOTER_MOTORS_FF);
        Builder.configurePIDController(topPID, false, new PIDConstants(Constants.Robot.SHOOTER_MOTORS_P, Constants.Robot.SHOOTER_MOTORS_I, Constants.Robot.SHOOTER_MOTORS_D), Constants.Robot.SHOOTER_MOTORS_IZ, Constants.Robot.SHOOTER_MOTORS_FF);
        
        bottomMotorEncoder = bottomMotor.getEncoder();
        topMotorEncoder = topMotor.getEncoder();

        if(Constants.PID_TEST_MODE){
            SmartDashboard.putNumber("shooterTestRPMInput",  0);    
        }
    }

    /**
     * Updates the inputs with the current values.
     * @param inputs The inputs to update
    */
    @Override
    public void updateInputs(ShooterInputs inputs){

        inputs.motorOneRPM = bottomMotorEncoder.getVelocity();
        inputs.motorTwoRPM = topMotorEncoder.getVelocity();
        inputs.setpointRPM = setpointRPM;
        inputs.isUpToSpeed = isUpToSpeed();
    }
    
    /**
     * Updates the setpoint RPM for the left and right shooter PIDs.
     * @param  setpointRPM    The desired setpoint in RPM
    */
    @Override
    public void updatePID(double setpointRPM){
        this.setpointRPM = setpointRPM;

        if(Constants.PID_TEST_MODE){
            double shooterTestRPMInput = SmartDashboard.getNumber("shooterTestRPMInput",  0);
            bottomPID.setReference(shooterTestRPMInput, CANSparkFlex.ControlType.kVelocity);
            topPID.setReference(shooterTestRPMInput, CANSparkFlex.ControlType.kVelocity);
        }else{
            bottomPID.setReference(setpointRPM, CANSparkFlex.ControlType.kVelocity);
            topPID.setReference(setpointRPM, CANSparkFlex.ControlType.kVelocity);
        }
    }

    /**
     * Stops the shooter by setting the PID setpoint to 0.
    */
    @Override
    public void stop(){
        updatePID(0.0);
    }

    /**
     * Sets the setpoint RPM.
    */
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
        double topMotorVelocity = topMotorEncoder.getVelocity();
        double bottomMotorVelocity = bottomMotorEncoder.getVelocity();
    
        boolean isLeftUpToSpeed = topMotorVelocity >= setpointRPM - Constants.Robot.SHOOTER_TOLERANCE_RPM
                && topMotorVelocity <= setpointRPM + Constants.Robot.SHOOTER_TOLERANCE_RPM;
    
        boolean isRightUpToSpeed = bottomMotorVelocity >= setpointRPM - Constants.Robot.SHOOTER_TOLERANCE_RPM
                && bottomMotorVelocity <= setpointRPM + Constants.Robot.SHOOTER_TOLERANCE_RPM;
    
        return isLeftUpToSpeed && isRightUpToSpeed;
    }
    
    @Override
    public void invertMotors(boolean inverted){
       // motorBottom.setInverted(inverted);
        topMotor.setInverted(inverted);
    }

}
