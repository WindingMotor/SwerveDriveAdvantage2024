// Written by WindingMotor as part of the wmlib2j library.

package frc.robot.wmlib2j.swerve;
import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.Constants.ModuleSettings;
import frc.robot.wmlib2j.util.Builder;

/**
 * Represents a real implementation of the IO module for the robot with standard MK4.
*/
public class IO_ModuleStandard implements IO_ModuleBase {

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
     * Constructor for the IO_ModuleStandard class.
     * @param settings The settings for the module.
    */
    public IO_ModuleStandard(ModuleSettings settings) {
        this.moduleSettings = settings;

        // Initialize the drive motor and its encoder
        driveMotor = Builder.createNeo(settings.DRIVE_ID, settings.DRIVE_INVERTED, 35);
        driveEncoder = Builder.createEncoder(driveMotor, Constants.MK4SDS.DRIVE_ENCODER_ROT_TO_METER, Constants.MK4SDS.DRIVE_ENCODER_RPM_TO_METER_PER_SEC);

        // Initialize the turn motor and its encoder
        turnMotor = Builder.createNeo(settings.TURN_ID, settings.TURN_INVERTED, 20);
        turnEncoder = Builder.createEncoder(turnMotor, Constants.MK4SDS.TURNING_ENCODER_ROT_TO_RAD, Constants.MK4SDS.TURNING_ENCODER_RPM_TO_RAD_PER_SEC);

        // Initialize the absolute encoder
        turnAbsoluteEncoder = new DutyCycleEncoder(settings.ABSOLUTE_ENCODER_ID);
        turnAbsoluteEncoder.setDutyCycleRange(1.0 / 4096.0, 4095.0 / 4096.0);

        // Burn flash on the motors
        driveMotor.burnFlash();
        turnMotor.burnFlash();

        // Initialize the RoboRio PID controllers
        driveRoboRioPID = new PIDController(0.45, 0.08, 0.001);
        turnRoboRioPID = new PIDController(0.25, 0.000001, 0.00025);
        turnRoboRioPID.enableContinuousInput(0.0, 2.0 * Math.PI);

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
        driveMotor.set(percent);
    }

    /**
     * Sets the turn motor speed.
     * @param percent The percentage of the turn output to set from -1.0 to 0.0.
    */
    @Override
    public void setTurnOutput(double percent){
        turnMotor.set(percent);
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
    @Override
    public void resetEncoders(){
        driveEncoder.setPosition(0.0);
        turnEncoder.setPosition(0.0);
        turnEncoder.setPosition(getAbsoluteEncoderRad());
    }

    /**
     * Updates the module SparkMax turn encoder position based on the current absolute encoder reading.
    */
    @Override
    public void updateTurnEncoder(){
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
        
        //turnMotor.set(turnRoboRioPID.calculate(turnEncoder.getPosition(), turnReference));
        // driveMotor.set(driveRoboRioPID.calculate(driveEncoder.getVelocity(), driveReference));
        //driveMotor.set(driveReference);

        turnMotor.set(turnRoboRioPID.calculate(turnEncoder.getPosition(), turnReference));
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
        double angle = turnAbsoluteEncoder.getAbsolutePosition();
        angle *= 2.0 * Math.PI;
        angle -= moduleSettings.ABSOLUTE_ENCODER_OFFSET;
        angle %= 2.0 * Math.PI;
        if (angle < 0) {
            angle += 2.0 * Math.PI;
        }
        return angle;
      }
      



}
