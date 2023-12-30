package frc.robot.wmlib2j.swerve;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.Constants.ModuleSettings;

/**
 * Represents a real implementation of the IO module for the robot.
*/
public class IO_ModuleReal implements IO_ModuleBase {

    // Store module settings
    private ModuleSettings moduleSettings;

    // Drive motor
    private CANSparkMax driveMotor;

    // Encoder for the drive motor
    private RelativeEncoder driveEncoder;

    // Turn motor
    private CANSparkMax turnMotor;

    // Encoder for the turn motor
    private RelativeEncoder turnEncoder;

    // Absolute encoder
    private DutyCycleEncoder turnAbsoluteEncoder;

    // PID controller for the drive motor
    private PIDController driveRoboRioPID;

    // PID controller for the turn motor
    private PIDController turnRoboRioPID;

    /**
    * Constructor for the IO_ModuleReal class.
    * @param settings The settings for the module.
    */
    public IO_ModuleReal(ModuleSettings settings) {
        this.moduleSettings = settings;

        // Initialize the drive motor and its encoder
        driveMotor = createMotor(settings.DRIVE_ID, settings.DRIVE_INVERTED, 35);
        driveEncoder = createEncoder(driveMotor, Constants.MK4SDS.DRIVE_ENCODER_ROT_TO_METER, Constants.MK4SDS.DRIVE_ENCODER_RPM_TO_METER_PER_SEC);

        // Initialize the turn motor and its encoder
        turnMotor = createMotor(settings.TURN_ID, settings.TURN_INVERTED, 20);
        turnEncoder = createEncoder(turnMotor, Constants.MK4SDS.TURNING_ENCODER_ROT_TO_RAD, Constants.MK4SDS.TURNING_ENCODER_RPM_TO_RAD_PER_SEC);

        // Initialize the absolute encoder
        turnAbsoluteEncoder = new DutyCycleEncoder(settings.ABSOLUTE_ENCODER_ID);
        turnAbsoluteEncoder.setDutyCycleRange(1.0 / 4096.0, 4095.0 / 4096.0);

        // Reset the encoder positions
        resetEncoders();

        // Burn flash on the motors
        driveMotor.burnFlash();
        turnMotor.burnFlash();

        // Initialize the RoboRio PID controllers
        driveRoboRioPID = new PIDController(0.45, 0.08, 0.001);
        turnRoboRioPID = new PIDController(0.25, 0.000001, 0.00025);
        turnRoboRioPID.enableContinuousInput(0.0, 2.0 * Math.PI);
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
        motor.setIdleMode(IdleMode.kBrake);
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
    private RelativeEncoder createEncoder(CANSparkMax motor, double positionConversionFactor, double velocityConversionFactor){
        RelativeEncoder encoder = motor.getEncoder();
        encoder.setPositionConversionFactor(positionConversionFactor);
        encoder.setVelocityConversionFactor(velocityConversionFactor);
        return encoder;
    }

    /**
    * Updates the inputs with the current values.
    * @param inputs The inputs to update.
    */
    @Override
    public void updateInputs(ModuleInputs inputs){

        // Drive motor inputs
        inputs.drivePositionMeters = driveEncoder.getPosition();
        inputs.driveVelocityMetersPerSec = driveEncoder.getVelocity();
        inputs.driveAppliedPercentage = driveMotor.getAppliedOutput();
        inputs.driveCurrentAmps = driveMotor.getOutputCurrent();
        inputs.driveTempCelsius = driveMotor.getMotorTemperature();

        // Turn motor inputs
        inputs.turnAbsolutePositionRad = getAbsoluteEncoderRad();
        inputs.turnPositionRad = turnEncoder.getPosition();
        inputs.turnVelocityRadPerSec = turnEncoder.getVelocity();
        inputs.turnAppliedPercentage = turnMotor.getAppliedOutput();
        inputs.turnCurrentAmps = turnMotor.getOutputCurrent();
        inputs.driveTempCelsius = turnMotor.getMotorTemperature();

    }

    /**
    * Sets the drive motor speed.
    * @param percent The percentage of the drive output to set from -1.0 to 0.0.
    */
    @Override
    public void setDriveOutput(double percent){
        driveMotor.set(0.0);
    }

    /**
    * Sets the turn motor speed.
    * @param percent The percentage of the turn output to set from -1.0 to 0.0.
    */
    @Override
    public void setTurnOutput(double percent){
        turnMotor.set(0.0);
    }

    /**
    * Stops the both the motors.
    */
    @Override
    public void stop(){
        driveMotor.set(0.0);
        turnMotor.set(0.0);
    }

    /**
    * Resets the positions of the encoders.
    */
    public void resetEncoders(){
        driveEncoder.setPosition(0.0);
        turnEncoder.setPosition(getAbsoluteEncoderRad());
    }

    /**
    * Sets the SparkMax PID references. The main way for controlling the motors.
    * @param driveReference The drive reference in meters per sec.
    * @param turnReference The turn reference in radians.
    */
    @Override
    public void setPIDReferences(double driveReference, double turnReference){
        //drivePID.setReference(2.0, CANSparkMax.ControlType.kVelocity);
        //turnPID.setReference(0.0, CANSparkMax.ControlType.kPosition);
        
        turnMotor.set(turnRoboRioPID.calculate(turnEncoder.getPosition(), turnReference));
       // driveMotor.set(driveRoboRioPID.calculate(driveEncoder.getVelocity(), driveReference));
       driveMotor.set(driveReference);
    }

    /**
    * Sets the module angle to zero and the drive velocity to zero.
    */
    public void setZero(){
        //drivePID.setReference(2.0, CANSparkMax.ControlType.kVelocity);
        //turnPID.setReference(0.0, CANSparkMax.ControlType.kPosition);
        
        turnMotor.set(turnRoboRioPID.calculate(turnEncoder.getPosition(), 0));
        driveMotor.set(driveRoboRioPID.calculate(driveEncoder.getVelocity(), 0));
    }

    /**
    * Gets the absolute encoder position in radians with the applied offset.
    * @return The position in radians.
    */
    public double getAbsoluteEncoderRad(){
        double angle;
    
        // Make sure position goes from 1 to 0 and convert to radians
        angle = 1 - turnAbsoluteEncoder.getAbsolutePosition();
        angle *= 2.0 * Math.PI;

        // Apply the offset in radians
        angle -= moduleSettings.ABSOLUTE_ENCODER_OFFSET.getRadians();

        return angle;
      }
    

}
