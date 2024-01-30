
package frc.robot.beluga.conveyor;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

/** An example command that uses an example subsystem. */
public class CMD_IndexerManual extends Command {

  private final SUB_Conveyor intexer;
  private final boolean run;


  public CMD_IndexerManual(SUB_Conveyor intexer, boolean run){
    this.intexer = intexer;
    this.run = run;
    addRequirements(intexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    if(run){
      intexer.setState(Constants.States.ConveyorState.INTAKE);
    }else{
      intexer.setState(Constants.States.ConveyorState.INTAKE_REV);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
    intexer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished(){
    return true;
  }

}
