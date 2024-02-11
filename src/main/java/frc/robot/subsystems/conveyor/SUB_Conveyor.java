
package frc.robot.subsystems.conveyor;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.States.ConveyorState;
import frc.robot.subsystems.conveyor.ConveyorInputsAutoLogged;
public class SUB_Conveyor extends SubsystemBase{

    private final IO_ConveyorBase io;

    public final ConveyorInputsAutoLogged inputs = new ConveyorInputsAutoLogged();

    private ConveyorState lastState;
    private ConveyorState state;

    public SUB_Conveyor(IO_ConveyorBase io){
        this.io = io;
        state = ConveyorState.OFF;
        lastState = ConveyorState.OFF;
    }

    @Override
    public void periodic(){
        // Update the inputs.
        io.updateInputs(inputs);

        // Process inputs and send to logger.
        Logger.processInputs("Conveyor", inputs);

        io.update(state.intakeSpeed, state.indexerSpeed);
    }

    public void setState(ConveyorState newState){
        lastState = state;
        state = newState;
    }

    public ConveyorState getState(){
        return state;
    }

    public ConveyorState getLastState(){
        return lastState;
    }

    public void stop(){
        io.stop();
    }

}