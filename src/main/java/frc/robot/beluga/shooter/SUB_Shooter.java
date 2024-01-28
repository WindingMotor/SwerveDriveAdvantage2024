
package frc.robot.beluga.shooter;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.States.ShooterState;

public class SUB_Shooter extends SubsystemBase{

    private final IO_ShooterBase io;

    private final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

    private ShooterState lastState;
    private ShooterState state;

    public SUB_Shooter(IO_ShooterBase io){
        this.io = io;
        this.lastState = ShooterState.OFF;
        this.state = ShooterState.OFF;
    }

    @Override
    public void periodic(){
        // Update the inputs.
        io.updateInputs(inputs);

        // Process inputs and send to logger.
        Logger.processInputs("Shooter", inputs);

        io.updatePID(state.rpm);
    }

    public void setState(ShooterState newState){
        if(newState != lastState){
            lastState = state;
            state = newState;
        }
    }

    public ShooterState getState(){
        return state;
    }

    public ShooterState getLastState(){
        return lastState;
    }

    public void stop(){
        io.stop();
    }

}