
package frc.robot.beluga.arm;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.wmlib2j.util.Builder;

public class IO_ArmReal implements IO_ArmBase{

    private CANSparkMax motorOne;       
    private CANSparkMax motorTwo;         
    
    private SparkAbsoluteEncoder motorOneAbsoluteEncoder;

    private ProfiledPIDController pid;

    private SlewRateLimiter slew;

    private double setpointPosition;

    private double currentAngle;

    private boolean isOnLowSide;



    public IO_ArmReal(){

        currentAngle = 0.0; 

        isOnLowSide = true;

        motorOne = Builder.createNeo(Constants.Beluga.ARM_MOTOR_LEAD_ID, Constants.Beluga.ARM_MOTOR_LEAD_INVERTED, 25);
        motorTwo = Builder.setNeoFollower(motorOne, Builder.createNeo(Constants.Beluga.ARM_MOTOR_FOLLOWER_ID, Constants.Beluga.ARM_MOTOR_FOLLOWER_INVERTED, 25));

        Builder.configureIdleMode(motorOne, false);
        Builder.configureIdleMode(motorTwo, false);

        pid = new ProfiledPIDController(
            Constants.Beluga.ARM_MOTORS_P,
            Constants.Beluga.ARM_MOTORS_I,
            Constants.Beluga.ARM_MOTORS_D,
            new TrapezoidProfile.Constraints(Constants.Beluga.ARM_MAX_VELOCITY, Constants.Beluga.ARM_MAX_ACCELERATION)
        );

        slew = new SlewRateLimiter(Constants.Beluga.ARM_SLEW_RATE_POS);

        motorOneAbsoluteEncoder = motorOne.getAbsoluteEncoder(Type.kDutyCycle);
        motorOneAbsoluteEncoder.setPositionConversionFactor(0.3 * 360);
        motorOneAbsoluteEncoder.setVelocityConversionFactor(0.3 * 360);
        motorOneAbsoluteEncoder.setZeroOffset(0);
        
        SmartDashboard.putNumber("armAngle", 0.0);
    }

    /**
     * Updates the inputs with the current values.
     * @param inputs The inputs to update.
    */
    @Override
    public void updateInputs(ArmInputs inputs){

        currentAngle = motorOneAbsoluteEncoder.getPosition();

        double simTestAngleInput = 15.0;

        Logger.recordOutput("Updated Arm Angle", 
            updateArmAngle(simTestAngleInput, 100.0).getFirst()
        );

        Logger.recordOutput("Updated Arm Boolean", 
            updateArmAngle(simTestAngleInput, 100.0).getSecond()
        );

        inputs.motorOnePositionDegrees = currentAngle;
        inputs.motorTwoPositionDegrees = currentAngle;
        inputs.setpointPosition = setpointPosition;
        // inputs.isAtSetpoint = Math.abs(motorOneEncoder.getPosition() - setpointPosition) < 0.1;

    }

    /**
     * Updates the arm angle based of current absolute encoder position and if its passed the wrap around point. 
     * @param  absoluteValue	The value of the absolute encoder in degrees
     * @param  wrapAroundPositionDegrees	The point where the absolute encoder wraps around in degrees
     * @return         The new arm angle and isOnLowSide boolean to apply in a Pair
     * @author         Alec
    */

    /**
     * Updates the arm angle based on the current absolute encoder position and if it's passed the wrap around point. 
     * @param  absoluteValue	The value of the absolute encoder in degrees
     * @param  wrapAroundPositionDegrees	The point where the absolute encoder wraps around in degrees
     * @return         The new arm angle and isOnLowSide boolean to apply in a Pair
    */
    public Pair<Double, Boolean> updateArmAngle(double absoluteValue, double wrapAroundPositionDegrees) {
        if (absoluteValue > wrapAroundPositionDegrees && isOnLowSide) {
            return new Pair<>(absoluteValue - 360, false);
        } else if (absoluteValue < wrapAroundPositionDegrees && !isOnLowSide) {
            return new Pair<>(absoluteValue + 360, true);
        } else {
            return new Pair<>(absoluteValue, isOnLowSide);
        }
    }

    /**
     * Converts the input value to degrees and applies the arm offset.
     * @param  input	The input value to be converted
     * @return         The converted value in degrees with arm offset applied
     * @deprecated     Use {@link #updateArmAngle(double)}
    */
    private double convertToDegrees(double relativeEncoderPosition){
        return relativeEncoderPosition * 360 - Constants.Beluga.ARM_OFFSET_DEGREES;
    }
    
    /**
     * Updates the PID controller with the new setpoint position.
     * @param  setpointPosition  The new setpoint position for the PID controller
    */
    @Override
    public void updatePID(double setpointPosition){

        /* Live debug code
        double armAngle = SmartDashboard.getNumber("armAngle",  0);
        if(armAngle > 110){
            armAngle = 110;
            SmartDashboard.putNumber("armAngle",110);
        }

        this.setpointPosition = setpointPosition;
        */

        motorOne.set(
            slew.calculate(
                pid.calculate(currentAngle, 25)
            )
        );
    }

    /**
     * Stops the arm motors.
    */
    @Override
    public void stop(){
        updatePID(0.0);
    }

    @Override
    public void setAngle(double angle){
        setpointPosition = angle;
    }


}
