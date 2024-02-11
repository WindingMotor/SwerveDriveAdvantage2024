package frc.robot.command.intake;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.beluga.arm.SUB_Arm;
import frc.robot.beluga.conveyor.SUB_Conveyor;
import frc.robot.Constants;
import java.util.function.Supplier;

/**
 * Command to control the intake process.
 */
public class CMD_Intake extends Command{

    private final SUB_Conveyor conveyor;
    private final SUB_Arm arm;
    private final Supplier<Boolean> manualCancel;
    private boolean isCommandDone = false;
    private Debouncer debouncer;

    /**
     * Constructs a new CMD_Intake command.
     * @param conveyor       The conveyor subsystem.
     * @param arm            The arm subsystem.
     * @param manualCancel   The supplier to determine if the command should be manually cancelled.
     */
    public CMD_Intake(SUB_Conveyor conveyor, SUB_Arm arm, Supplier<Boolean> manualCancel){
        this.conveyor = conveyor;
        this.arm = arm;
        this.manualCancel = manualCancel;
        debouncer = new Debouncer(0.025, Debouncer.DebounceType.kRising);

        addRequirements(conveyor, arm);
    }

    /**
     * When command starts reset the isCommandDone flag, report to the
     * driver station that the command is running, and set the robot subsystems to intake mode.
    */
    @Override
    public void initialize(){
        isCommandDone = false;
        DriverStation.reportWarning("[init] CMD_Intake running", false);
        conveyor.setState(Constants.States.ConveyorState.INTAKE);
        arm.setState(Constants.States.ArmState.INTAKE);
    }

    @Override
    public void execute(){
        // If the indexer sensor is triggered, end the command
        if(debouncer.calculate(conveyor.inputs.indexerInitalSensorState)){
            isCommandDone = true;
        }
    }

    /**
     * Sets conveyor and arm states to IDLE when command ends.
     * @param  interrupted   Whether the command was interrupted
    */
    @Override
    public void end(boolean interrupted){
        conveyor.setState(Constants.States.ConveyorState.OFF);
        arm.setState(Constants.States.ArmState.IDLE);
    }

    @Override
    public boolean isFinished(){
        if(isCommandDone || manualCancel.get()){
            return true;
        }else{
            return false;
            
        }
    }

}