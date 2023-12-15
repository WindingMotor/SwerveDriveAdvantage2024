package frc.robot.wmlib2.swerve;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
        SwerveModuleState optimizedState = SwerveModuleState.optimize(newState, getModuleAngle());

        if (Constants.currentMode == Constants.RobotMode.REAL){
            // Set both PIDs references, (Velocity & Position), to tell the module's SparkMaxes where to go
            io.setPIDReferences(optimizedState.speedMetersPerSecond, optimizedState.angle.getRadians());
        }else{
            // Set output speeds using RoboRio PID controllers for simulation
            io.setDriveOutput(simTurnPID.calculate(getModuleAngle().getRadians(), optimizedState.angle.getRadians()));

            // optimizedState.speedMetersPerSecond *= cos(simTurnPID.positionError) // Update velocity based off turn error

            double velocityRadPerSec = optimizedState.speedMetersPerSecond / Constants.MK4SDS.WHEEL_DIAMETER;
            io.setTurnOutput(simDrivePID.calculate(inputs.driveVelocityRadPerSec, velocityRadPerSec));
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
        return new Rotation2d(MathUtil.angleModulus(inputs.turnAbsolutePositionRad));
    }

    // Returns the current drive position of the module in meters.
    public double getModulePositionMeters(){
        return inputs.drivePositionRad * Constants.MK4SDS.WHEEL_DIAMETER;
    }

    // Returns the current drive velocity of the module in meters per second.
    public double getVelocityMetersPerSec(){
        return inputs.driveVelocityRadPerSec * Constants.MK4SDS.WHEEL_DIAMETER;
    }

    // Returns the module position (turn angle and drive position).
    public SwerveModulePosition getModulePosition(){
        return new SwerveModulePosition(getModulePositionMeters(), getModuleAngle());
    }

    // Returns the module state (turn angle and drive velocity).
    public SwerveModuleState getModuleState(){
        return new SwerveModuleState(getModulePositionMeters(), getModuleAngle());
    }

    public void stop(){
        io.stop();
    }

}

