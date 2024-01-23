
package frc.robot.wmlib2j.util;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

/**
 * The Builder class provides static methods to create different types of hardware.
 * Example usage:
 * CANSparkMax neoMotor = Builder.createNeo(1, false, 40);
 * RelativeEncoder encoder = Builder.createEncoder(neoMotor, 0.5, 1.0);
 * SparkPIDController pidController = new SparkPIDController();
 * Builder.configurePIDController(pidController, false);
*/
public class Builder {
    
    /**
     * Creates a motor with the given CAN ID and inversion setting.
     * @param id The CAN ID of the motor.
     * @param inverted Whether the direction motor is inverted.
     * @param currentLimit The current limit for the motor.
     * @return The created Neo motor.
    */
    public static CANSparkMax createNeo(int id, boolean inverted, int currentLimit){
        CANSparkMax motor = new CANSparkMax(id, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kCoast);
        motor.setSmartCurrentLimit(currentLimit);
        motor.setInverted(inverted);
        return motor;
    }

    /**
     * Creates a Vortex motor with the given parameters.
     * @param  id            The unique ID of the motor
     * @param  inverted      true if the motor should be inverted, false otherwise
     * @param  currentLimit  The current limit for the motor
     * @return               The created Vortex motor
    */
    public static CANSparkFlex createVortex(int id, boolean inverted, int currentLimit){
        CANSparkFlex motor = new CANSparkFlex(id, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setIdleMode(com.revrobotics.CANSparkBase.IdleMode.kBrake);
        motor.setSmartCurrentLimit(currentLimit);
        motor.setInverted(inverted);
        return motor;
    }

    /**
     * Creates an encoder with the given motor and conversion factors.
     * @param motor The motor to bind the encoder to.
     * @param positionConversionFactor The conversion factor for the position.
     * @param velocityConversionFactor The conversion factor for the velocity.
     * @return The created encoder.
    */
    public static RelativeEncoder createEncoder(CANSparkBase motor, double positionConversionFactor, double velocityConversionFactor){
        RelativeEncoder encoder = motor.getEncoder();
        encoder.setPositionConversionFactor(positionConversionFactor);
        encoder.setVelocityConversionFactor(velocityConversionFactor);
        return encoder;
    }

    /**
     * Configures a Spark PID controller.
     * @param  pid          the SparkPIDController to be configured
     * @param  isTurnPID    true if it is a turn PID, false otherwise
     */
    public static void configurePIDController(SparkPIDController pid, boolean isTurnPID){
        pid.setP(0.0);
        pid.setI(0.0);
        pid.setD(0.0);
        pid.setOutputRange(-1.0, 1.0);
        if(isTurnPID){
            pid.setOutputRange(0.0, 2 * Math.PI);
        }
    }



}
