package frc.robot.wmlib2.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants.MK4SDS;
import frc.robot.Constants.ModuleSettings;

// Abstracted from IO_IntakeBase, contains the code to interface with the hardware.
public class IO_ModuleReal implements IO_ModuleBase{

    private ModuleSettings settings;

    // Create the drive motor
    private CANSparkMax driveMotor = new CANSparkMax(settings.driveID, MotorType.kBrushless);

    // Create the drive encoder
    private RelativeEncoder driveEncoder = driveMotor.getEncoder();
    
    // Create the turn motor
    private CANSparkMax turnMotor = new CANSparkMax(settings.turnID, MotorType.kBrushless);

    // Create the turn encoder
    private RelativeEncoder turnEncoder = turnMotor.getEncoder();

    // Create the absolute encoder
    private DutyCycleEncoder turnAbsoluteEncoder = new DutyCycleEncoder(settings.absoluteEncoderID);

    // Create module PIDs
    private SparkMaxPIDController turnPID = turnMotor.getPIDController();
    private SparkMaxPIDController drivePID = driveMotor.getPIDController();

    public IO_ModuleReal(ModuleSettings settings){

        this.settings = settings;

        driveMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.setSmartCurrentLimit(45);
        driveMotor.setInverted(false);
        driveMotor.burnFlash();

        driveEncoder.setPositionConversionFactor(MK4SDS.DRIVE_ROT_2_METER);
        driveEncoder.setVelocityConversionFactor(MK4SDS.DRIVE_RPM_2_MPS);

        turnMotor.setIdleMode(IdleMode.kBrake);
        turnMotor.setSmartCurrentLimit(25);
        turnMotor.setInverted(false);
        turnMotor.burnFlash();

        turnEncoder.setPositionConversionFactor(MK4SDS.TURN_ROT_2_RAD);
        turnEncoder.setVelocityConversionFactor(MK4SDS.TURN_RPM_2_RADPS);

        turnAbsoluteEncoder.setDutyCycleRange(1.0 / 4096.0, 4095.0 / 4096.0);

        turnPID.setP(MK4SDS.TURN_MODULE_PID_P);
        turnPID.setI(MK4SDS.TURN_MODULE_PID_I);
        turnPID.setD(MK4SDS.TURN_MODULE_PID_D);
        turnPID.setOutputRange(-1.0, 1.0);
        turnPID.setPositionPIDWrappingEnabled(true);
        turnPID.setPositionPIDWrappingMinInput(-Math.PI);
        turnPID.setPositionPIDWrappingMaxInput(Math.PI);
        turnPID.setFeedbackDevice(turnEncoder);

        drivePID.setP(MK4SDS.DRIVE_MODULE_PID_P);
        drivePID.setI(MK4SDS.DRIVE_MODULE_PID_I);
        drivePID.setD(MK4SDS.DRIVE_MODULE_PID_D);
        drivePID.setOutputRange(-1.0, 1.0);
        drivePID.setFeedbackDevice(driveEncoder);
    }

    // Update inputs with current values
    @Override
    public void updateInputs(ModuleInputs inputs){

        inputs.drivePositionRad = driveEncoder.getPosition();
        inputs.driveVelocityRadPerSec = driveEncoder.getVelocity();
        inputs.driveAppliedPercentage = driveMotor.getAppliedOutput();
        inputs.driveCurrentAmps = driveMotor.getOutputCurrent();
        inputs.driveTempCelsius = driveMotor.getMotorTemperature();

        // Get absolute position -> Convert to radians -> Wrap from -pi to pi -> Subtract offset -> Wrap again
        inputs.turnAbsolutePositionRad = MathUtil.angleModulus(
            Rotation2d.fromDegrees(turnAbsoluteEncoder.getAbsolutePosition()).minus(settings.absoluteEncoderOffset).getRadians()
        );

        inputs.turnPositionRad = turnEncoder.getPosition();
        inputs.turnVelocityRadPerSec = turnEncoder.getVelocity();
        inputs.turnAppliedPercentage = turnMotor.getAppliedOutput();
        inputs.turnCurrentAmps = turnMotor.getOutputCurrent();
        inputs.driveTempCelsius = turnMotor.getMotorTemperature();
    }

    @Override
    public void setDriveOutput(double percent){
        driveMotor.set(percent);
    }

    @Override
    public void setTurnOutput(double percent){
        turnMotor.set(percent);
    }

    @Override
    public void stop(){
        driveMotor.set(0.0);
        turnMotor.set(0.0);
    }

    @Override
    public void setPIDReferences(double driveReference, double turnReference){
        drivePID.setReference(driveReference, CANSparkMax.ControlType.kVelocity);
        turnPID.setReference(turnReference, CANSparkMax.ControlType.kPosition);
    }
    
}
