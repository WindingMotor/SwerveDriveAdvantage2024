package frc.robot.wmlib2j.swerve;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.Constants.ModuleSettings;

/**
 * Represents an individual swerve module.
 * It contains methods for controlling the module's state and retrieving its current state.
*/
public class Module {

    private final IO_ModuleBase io;
    private final ModuleSettings settings;

    private final IO_ModuleBase.ModuleInputs inputs = new IO_ModuleBase.ModuleInputs();

    private final PIDController simDrivePID = new PIDController(0.1, 0.0, 0.0);
    private final PIDController simTurnPID = new PIDController(0.1, 0.0, 0.0);

    /**
     * Constructor for the Module class.
     * @param io The IO module base.
     * @param settings The module settings.
    */
    public Module(IO_ModuleBase io, ModuleSettings settings){
        this.io = io;
        this.settings = settings;
    }

    /**
     * Returns the new optimized state while setting motor speeds via setPIDReferences.
     * @param newState The new state of the module.
     * @return The optimized state of the module.
    */
    public SwerveModuleState runWithState(SwerveModuleState newState){

        SwerveModuleState optimizedState = SwerveModuleState.optimize(newState, getModuleState().angle);

        if (Constants.CURRENT_MODE == Constants.RobotMode.REAL){
            io.setPIDReferences(optimizedState.speedMetersPerSecond, optimizedState.angle.getRadians());
        }else{
            double turnError = simTurnPID.calculate(getModuleAngle().getRadians(), optimizedState.angle.getRadians());
            io.setDriveOutput(turnError);

            double velocityRadPerSec = optimizedState.speedMetersPerSecond / Constants.MK4SDS.WHEEL_DIAMETER_METERS;
            io.setTurnOutput(simDrivePID.calculate(inputs.driveVelocityMetersPerSec, velocityRadPerSec));
        }

        return optimizedState;
    }

    public void periodic(){
        // Update the inputs.
        io.updateInputs(inputs);

        // Process inputs and send to logger.
        Logger.processInputs(settings.MODULE_NAME, inputs);
    }


    /**
     * Returns the current angle of the module.
     * @return The current turn angle of the module in radians.
    */
    public Rotation2d getModuleAngle(){
        return new Rotation2d(inputs.turnPositionRad);
    }

    /**
     * Returns the current drive position, distance, of the module.
     * @return The current drive position of the module in meters.
    */
    public double getModulePositionMeters(){
        return inputs.drivePositionMeters;
    }

    /**
     * Returns the current wheel velocity of the module.
     * @return The current drive motor velocity of the module in meters per second.
    */
    public double getModuleVelocityMetersPerSec(){
        return inputs.driveVelocityMetersPerSec;
    }

    /**
     * Returns the module position, (turn angle and drive position).
     * @return The module position.
     */
    public SwerveModulePosition getModulePosition(){
        return new SwerveModulePosition(getModulePositionMeters(), getModuleAngle());
    }

    /**
     * Returns the module state, (turn angle and drive velocity).
     * @return The module state.
     */
    public SwerveModuleState getModuleState(){
        return new SwerveModuleState(getModuleVelocityMetersPerSec(), getModuleAngle());
    }

    /**
     * Stops the module.
    */
    public void stop(){
        io.stop();
    }

}

