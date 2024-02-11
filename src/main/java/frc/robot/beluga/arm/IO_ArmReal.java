
package frc.robot.beluga.arm;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkAbsoluteEncoder.Type;

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
    
 //   private RelativeEncoder motorOneEncoder;
   // private RelativeEncoder motorTwoEncoder;

    private PIDController armPID;
    //private SimpleMotoArmFeedforwardrFeedforward feedforward;


    private double setpointPosition;

    private SparkPIDController armSparkPID;

    private SparkAbsoluteEncoder motorOneAbsoluteEncoder;

    //private ProfiledPIDController armPIDController;

    private ArmFeedforward armFeedforward;

    private SlewRateLimiter slew;

    private double currentArmAngle;

    private boolean isOnLowSide;

    public IO_ArmReal(){

        currentArmAngle = 0.0; 

        isOnLowSide = true;

        motorOne = Builder.createNeo(Constants.Beluga.ARM_MOTOR_LEAD_ID, Constants.Beluga.ARM_MOTOR_LEAD_INVERTED, 25);
        motorTwo = Builder.setNeoFollower(motorOne, Builder.createNeo(Constants.Beluga.ARM_MOTOR_FOLLOWER_ID, Constants.Beluga.ARM_MOTOR_FOLLOWER_INVERTED, 25));

       // motorOneEncoder = Builder.createEncoder(motorOne, 0.0105, 0.0105);
       // motorTwoEncoder = Builder.createEncoder(motorTwo, 0.0105, 0.0105);

  
       armPID = new PIDController(Constants.Beluga.ARM_MOTORS_P, Constants.Beluga.ARM_MOTORS_I, Constants.Beluga.ARM_MOTORS_D);
        //feedforward = new SimpleMotorFeedforward(0.98, 0.03);
        //feedforward = new ArmFeedforward(setpointPosition, setpointPosition, setpointPosition)

        //Builder.configurePIDController(motorOnePID, false, new PIDConstants(Constant s.Beluga.ARM_MOTORS_P, Constants.Beluga.ARM_MOTORS_I, Constants.Beluga.ARM_MOTORS_D));
        //Builder.configurePIDController(motorTwoPID, false, new PIDConstants(Constants.Beluga.ARM_MOTORS_P, Constants.Beluga.ARM_MOTORS_I, Constants.Beluga.ARM_MOTORS_D));
        
        Builder.configureIdleMode(motorOne, false);
        Builder.configureIdleMode(motorTwo, false);

        slew = new SlewRateLimiter(Constants.Beluga.ARM_SLEW_RATE_POS);

        //Builder.configurePIDController(armSparkPID, false, new PIDConstants(Constants.Beluga.ARM_MOTORS_P, Constants.Beluga.ARM_MOTORS_I, Constants.Beluga.ARM_MOTORS_D));


        armSparkPID = motorOne.getPIDController();
        
        motorOneAbsoluteEncoder = motorOne.getAbsoluteEncoder(Type.kDutyCycle);

        motorOneAbsoluteEncoder.setPositionConversionFactor(0.3 * 360);
        motorOneAbsoluteEncoder.setVelocityConversionFactor(0.3 * 360);
        motorOneAbsoluteEncoder.setZeroOffset(0);

        
        armSparkPID.setFeedbackDevice(motorOneAbsoluteEncoder);
        armSparkPID.setPositionPIDWrappingEnabled(false);
        armSparkPID.setP(0.05);
        
        
        SmartDashboard.putNumber("armAngle", 0.0);


        //armFeedforward = new ArmFeedforward(0.0, 1.22, 0.03);
       // armFeedforward = new ArmFeedforward(0.0, 1.22, 0.03, 0.00174);

       // armPIDController = new ProfiledPIDController(Constants.Beluga.ARM_MOTORS_P, Constants.Beluga.ARM_MOTORS_I, Constants.Beluga.ARM_MOTORS_D, new TrapezoidProfile.Constraints(Constants.Beluga.ARM_MAX_VELOCITY, Constants.Beluga.ARM_MAX_ACCLERATION));
    }

    /**
     * Updates the inputs with the current values.
     * @param inputs The inputs to update.
    */
    @Override
    public void updateInputs(ArmInputs inputs){


        currentArmAngle = motorOneAbsoluteEncoder.getPosition();

    // Check if the arm has passed the wrap-around point while moving in the opposite direction
    if (currentArmAngle >   106 && isOnLowSide) {
        isOnLowSide = false;
    } else if (currentArmAngle <   5 && !isOnLowSide) {
        isOnLowSide = true;
    }

        SmartDashboard.putBoolean("ISONLOWSIDE", isOnLowSide);

        inputs.motorOnePositionDegrees = currentArmAngle;
        inputs.motorTwoPositionDegrees = -1.0;
        inputs.setpointPosition = setpointPosition;
       // inputs.isAtSetpoint = Math.abs(motorOneEncoder.getPosition() - setpointPosition) < 0.1;

        Mechanism2d mechanism = new Mechanism2d(3, 3);
        Logger.recordOutput("armMechanism", mechanism);
    }

    /**
     * Converts the input value to degrees and applies the arm offset.
     * @param  input	The input value to be converted
     * @return         The converted value in degrees with arm offset applied
    */
   // private double convertToDegrees(double input){
  //      return motorOneEncoder.getPosition() * 360 - Constants.Beluga.ARM_OFFSET_DEGREES;
  //  }

    /**
     * Updates the PID controller with the new setpoint position.
     * @param  setpointPosition  The new setpoint position for the PID controller
        */
        @Override
    public void updatePID(double setpointPosition){

        /* 
        double armAngle = SmartDashboard.getNumber("armAngle",  0);
        if(armAngle > 110){
            armAngle = 110;
            SmartDashboard.putNumber("armAngle",110);
        }

        this.setpointPosition = setpointPosition;
*/


    motorOne.set(
        slew.calculate(
           armPID.calculate(currentArmAngle, 25)
        )
    );

      //double calculated = -armPIDController.calculate(motorOneAbsoluteEncoder.getPosition(), 25);
        
     // motorOne.set(calculated);

     
        //SmartDashboard.putNumber("ARM NEW SETPOINT", armAngle - Constants.Beluga.ARM_OFFSET_DEGREES);

       // armSparkPID.setReference(15, CANSparkMax.ControlType.kPosition);
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
