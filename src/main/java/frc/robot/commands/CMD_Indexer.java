// Written by WindingMotor as part of the wmlib2j library.

package frc.robot.commands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.States.ConveyorState;
import frc.robot.subsystems.conveyor.SUB_Conveyor;

/**
 * Class to handle shooting commands.
 */
public class CMD_Indexer extends Command{

    private final SUB_Conveyor conveyor;
    private ConveyorState state;

    public CMD_Indexer(SUB_Conveyor conveyor){
        this.conveyor = conveyor;
        addRequirements(conveyor);
        state = Constants.States.ConveyorState.OFF;
    }

    public CMD_Indexer(SUB_Conveyor conveyor, ConveyorState newState){
        this.conveyor = conveyor;
        addRequirements(conveyor);
        state = newState;
    }

    @Override
    public void initialize(){
        DriverStation.reportWarning("[init] CMD_Indexer set new state of" + state, false);
        conveyor.setState(state);
    }

    @Override
    public boolean isFinished(){ return true; }
}
