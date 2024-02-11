
package frc.robot.beluga.conveyor;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

/** An example command that uses an example subsystem. */
public class CMD_IndexerManual extends Command {

  private final SUB_Conveyor conveyor;
  private final boolean isReversed;


  public CMD_IndexerManual(SUB_Conveyor conveyor, boolean isReversed){
    this.conveyor = conveyor;
    this.isReversed = isReversed;
    addRequirements(conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    if(isReversed){
      conveyor.setState(Constants.States.ConveyorState.EJECT);
    }else{
      conveyor.setState(Constants.States.ConveyorState.INTAKE);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
    conveyor.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished(){
    return true;
  }

}
