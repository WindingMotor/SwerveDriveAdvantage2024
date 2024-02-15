// Written by WindingMotor, 2024, Crescendo

package frc.robot.subsystems.conveyor;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

/* Manually stop the indexer. */
public class CMD_IndexerStop extends Command {

  private final SUB_Conveyor conveyor;

  public CMD_IndexerStop(SUB_Conveyor conveyor){
    this.conveyor = conveyor;
    addRequirements(conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    conveyor.setState(Constants.States.ConveyorState.OFF);
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