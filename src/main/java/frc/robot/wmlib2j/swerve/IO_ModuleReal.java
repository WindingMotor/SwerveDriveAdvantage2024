package frc.robot.wmlib2j.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.MK4SDS;
import frc.robot.Constants.ModuleSettings;

// Abstracted from IO_IntakeBase, contains the code to interface with the hardware.
public class IO_ModuleReal implements IO_ModuleBase{

    private ModuleSettings settings;

    // Create the drive motor
    private CANSparkMax driveMotor;

    // Create the drive encoder
    private RelativeEncoder driveEncoder;
    
    // Create the turn motor
    private CANSparkMax turnMotor;

    // Create the turn encoder
    private RelativeEncoder turnEncoder;

    // Create the absolute encoder
    private DutyCycleEncoder turnAbsoluteEncoder;

    // Create module PIDs
    private SparkMaxPIDController turnPID;
    private SparkMaxPIDController drivePID;

    private PIDController turnRoboRioPID;
    private PIDController driveRoboRioPID;

    public IO_ModuleReal(ModuleSettings settings){

        this.settings = settings;

        driveMotor = new CANSparkMax(settings.driveID, MotorType.kBrushless);
        driveMotor.restoreFactoryDefaults();
        driveEncoder = driveMotor.getEncoder();

        turnMotor = new CANSparkMax(settings.turnID, MotorType.kBrushless);
        turnMotor.restoreFactoryDefaults();
        turnEncoder = turnMotor.getEncoder();

        turnAbsoluteEncoder = new DutyCycleEncoder(settings.absoluteEncoderID);

        //turnPID = turnMotor.getPIDController();
        //drivePID = driveMotor.getPIDController();

        driveMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.setSmartCurrentLimit(35);
        driveMotor.setInverted(settings.driveInverted);

        driveEncoder.setPositionConversionFactor(MK4SDS.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(MK4SDS.kDriveEncoderRPM2MeterPerSec);
    

        turnMotor.setIdleMode(IdleMode.kBrake);
        turnMotor.setSmartCurrentLimit(15);
        turnMotor.setInverted(settings.turnInverted);
 
        // Change conversion factors for neo turning encoder - should be in radians!
        turnEncoder.setPositionConversionFactor(MK4SDS.kTurningEncoderRot2Rad);
        turnEncoder.setVelocityConversionFactor(MK4SDS.kTurningEncoderRPM2RadPerSec);

        turnAbsoluteEncoder.setDutyCycleRange(1.0 / 4096.0, 4095.0 / 4096.0);

        /* 
        turnPID.setP(MK4SDS.TURN_MODULE_PID_P);
        turnPID.setI(MK4SDS.TURN_MODULE_PID_I);
        turnPID.setD(MK4SDS.TURN_MODULE_PID_D);
        turnPID.setOutputRange(-1.0, 1.0);
        turnPID.setPositionPIDWrappingEnabled(true);
        turnPID.setPositionPIDWrappingMinInput(0.0);
        turnPID.setPositionPIDWrappingMaxInput( 2.0 * Math.PI);
        turnPID.setFeedbackDevice(turnEncoder);

        drivePID.setP(MK4SDS.DRIVE_MODULE_PID_P);
        drivePID.setI(MK4SDS.DRIVE_MODULE_PID_I);
        drivePID.setD(MK4SDS.DRIVE_MODULE_PID_D);
        drivePID.setOutputRange(-1.0, 1.0);
        drivePID.setFeedbackDevice(driveEncoder);
        */

        driveEncoder.setPosition(0.0);
        turnEncoder.setPosition(getAbsoluteEncoderRad());

        driveMotor.burnFlash();
        turnMotor.burnFlash();

        driveRoboRioPID = new PIDController(0.45, 0.08, 0.001);
        turnRoboRioPID = new PIDController(0.25, 0.000001, 0.00025);

        turnRoboRioPID.enableContinuousInput(0.0, 2.0 * Math.PI);
   



    }

    // Update inputs with current values
    @Override
    public void updateInputs(ModuleInputs inputs){

        inputs.drivePositionMeters = driveEncoder.getPosition();
        inputs.driveVelocityMetersPerSec = driveEncoder.getVelocity();
        inputs.driveAppliedPercentage = driveMotor.getAppliedOutput();
        inputs.driveCurrentAmps = driveMotor.getOutputCurrent();
        inputs.driveTempCelsius = driveMotor.getMotorTemperature();

        // Get absolute position -> Convert to radians -> Wrap from -pi to pi -> Subtract offset -> Wrap again
        //inputs.turnAbsolutePositionRad = MathUtil.angleModulus(
        //    Rotation2d.fromDegrees(turnAbsoluteEncoder.getAbsolutePosition()).minus(settings.absoluteEncoderOffset).getRadians()
        //);

        inputs.turnAbsolutePositionRad = getAbsoluteEncoderRad();

        inputs.turnPositionRad = turnEncoder.getPosition();
        inputs.turnVelocityRadPerSec = turnEncoder.getVelocity();
        inputs.turnAppliedPercentage = turnMotor.getAppliedOutput();
        inputs.turnCurrentAmps = turnMotor.getOutputCurrent();
        inputs.driveTempCelsius = turnMotor.getMotorTemperature();
    }

    @Override
    public void setDriveOutput(double percent){
        driveMotor.set(0.0);
    }

    @Override
    public void setTurnOutput(double percent){
        turnMotor.set(0.0);
    }

    @Override
    public void stop(){
        driveMotor.set(0.0);
        turnMotor.set(0.0);
    }

    @Override
    public void setPIDReferences(double driveReference, double turnReference){
        //drivePID.setReference(2.0, CANSparkMax.ControlType.kVelocity);
        //turnPID.setReference(0.0, CANSparkMax.ControlType.kPosition);
        //driveMotor.set(0.1);
        
        //turnMotor.set(turnRoboRioPID.calculate(turnEncoder.getPosition(), Math.toRadians(45.0)));
        //driveMotor.set(driveRoboRioPID.calculate(driveEncoder.getVelocity(), 1.0));

        turnMotor.set(turnRoboRioPID.calculate(turnEncoder.getPosition(), turnReference));
       // driveMotor.set(driveRoboRioPID.calculate(driveEncoder.getVelocity(), driveReference));
       driveMotor.set(driveReference);
       

    }

    public double getAbsoluteEncoderRad(){

        //  Make angle variable
        double angle;
    
        // Get encoder absolute position goes from 1 to 0
        angle = 1 - turnAbsoluteEncoder.getAbsolutePosition();
    
        // Convert into radians
        angle *= 2.0 * Math.PI;
        //angle -= (SmartDashboard.getNumber(moduleName + " ABE Manual", 0) / 180.0) * Math.PI;
        //System.out.println("WARNING: " + moduleName + " is at " + SmartDashboard.getNumber(moduleName + " ABE Manual", 0));
        angle -= settings.absoluteEncoderOffset.getRadians();

        return angle;
        
      }
    

}
