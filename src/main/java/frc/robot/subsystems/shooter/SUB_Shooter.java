
package frc.robot.subsystems.shooter;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.States.ShooterState;
import frc.robot.subsystems.shooter.ShooterInputsAutoLogged;

public class SUB_Shooter extends SubsystemBase{

    private final IO_ShooterBase io;

    public final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

    private ShooterState lastState;
    private ShooterState state;
    private double setpointRPM;

    public SUB_Shooter(IO_ShooterBase io){
        this.io = io;
        this.lastState = ShooterState.OFF;
        this.state = ShooterState.OFF;
        this.setpointRPM = 0.0;
    }

    @Override
    public void periodic(){
        // Update the inputs
        io.updateInputs(inputs);

        // Process inputs and send to logger
        Logger.processInputs("Shooter", inputs);

        setpointRPM = state.rpm;
        io.updatePID(setpointRPM);
    }

    public void setState(ShooterState newState){
        lastState = state;
        state = newState;
    }

    public ShooterState getState(){
        return state;
    }

    public ShooterState getLastState(){
        return lastState;
    }

    public void updateSetpointRPM(double newSetpointRPM){
        this.setpointRPM = newSetpointRPM; 
    }

    public void stop(){
        io.stop();
    }

    public void invertMotors(boolean inverted){
        io.invertMotors(inverted);
    }

}