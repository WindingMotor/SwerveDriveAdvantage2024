package frc.robot.wmlib2j.swerve;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.Constants.ModuleSettings;

public class Module {

    private final IO_ModuleBase io;
    private final ModuleSettings settings;

    private final IO_ModuleBase.ModuleInputs inputs = new IO_ModuleBase.ModuleInputs();

    private final PIDController simDrivePID = new PIDController(0.1, 0.0, 0.0);
    private final PIDController simTurnPID = new PIDController(0.1, 0.0, 0.0);

    public Module(IO_ModuleBase io, ModuleSettings settings){
        this.io = io;
        this.settings = settings;
    }

    // Returns the new optimized state of the module while sending motor voltage commands.
    public SwerveModuleState runWithState(SwerveModuleState newState){

        // Optimize the desired module state based on the current module angle.
        SwerveModuleState optimizedState = SwerveModuleState.optimize(newState, getModuleState().angle);

        if (Constants.currentMode == Constants.RobotMode.REAL){
            // Set both PIDs references, (Velocity & Position), to tell the module's SparkMaxes where to go
            io.setPIDReferences(optimizedState.speedMetersPerSecond, optimizedState.angle.getRadians());
            
            //SmartDashboard.putNumber( settings.moduleName + " Drive REF", optimizedState.speedMetersPerSecond);
            //SmartDashboard.putNumber( settings.moduleName + " Turn REF", optimizedState.angle.getRadians());
        }else{
            // Set output speeds using RoboRio PID controllers for simulation
            io.setDriveOutput(simTurnPID.calculate(getModuleAngle().getRadians(), optimizedState.angle.getRadians()));

            // optimizedState.speedMetersPerSecond *= cos(simTurnPID.positionError) // Update velocity based off turn error

            double velocityRadPerSec = optimizedState.speedMetersPerSecond / Constants.MK4SDS.kWheelDiameterMeters;
            io.setTurnOutput(simDrivePID.calculate(inputs.driveVelocityMetersPerSec, velocityRadPerSec));

        }

        return optimizedState;
    }

    public void periodic(){
        // Update the inputs.
        io.updateInputs(inputs);

        // Process inputs and send to logger.
        Logger.processInputs(settings.moduleName, inputs);
    }

    // Returns the current turn angle of the module.
    public Rotation2d getModuleAngle(){
        return new Rotation2d(inputs.turnPositionRad);
    }

    // Returns the current drive position of the module in meters.
    public double getModulePositionMeters(){
        return inputs.drivePositionMeters;
    }

    // Returns the current drive velocity of the module in meters per second.
    public double getModuleVelocityMetersPerSec(){
        return inputs.driveVelocityMetersPerSec;
    }

    // Returns the module position (turn angle and drive position).
    public SwerveModulePosition getModulePosition(){
        return new SwerveModulePosition(getModulePositionMeters(), getModuleAngle());
    }

    // Returns the module state (turn angle and drive velocity).
    public SwerveModuleState getModuleState(){
        return new SwerveModuleState(getModuleVelocityMetersPerSec(), getModuleAngle());
    }

    public void stop(){
        io.stop();
    }

}

