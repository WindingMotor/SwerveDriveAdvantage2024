// Written by WindingMotor as part of the wmlib2j library.

package frc.robot.commands;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.SUB_Arm;
import frc.robot.subsystems.conveyor.SUB_Conveyor;

import java.util.function.Supplier;

/**
 * Command to control the intake process.
 */
public class CMD_Eject extends Command{

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
    public CMD_Eject(SUB_Conveyor conveyor, SUB_Arm arm, Supplier<Boolean> manualCancel){
        this.conveyor = conveyor;
        this.arm = arm;
        this.manualCancel = manualCancel;
        debouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

        addRequirements(conveyor, arm);
    }

    /**
     * When command starts reset the isCommandDone flag, report to the
     * driver station that the command is running, and set the robot subsystems to eject mode.
    */
    @Override
    public void initialize(){
        isCommandDone = false;
        DriverStation.reportWarning("[init] CMD_Eject running", false);
        conveyor.setState(Constants.States.ConveyorState.EJECT);
        arm.setState(Constants.States.ArmState.INTAKE);
    }

    /**
     * Every cycle check if the intake sensor is triggered. Once donut
     * leaves the intake set the isCommandDone flag to true.
    */
    @Override
    public void execute(){
        if(debouncer.calculate(!conveyor.inputs.intakeInitalSensorState)){
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
