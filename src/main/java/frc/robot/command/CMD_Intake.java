
package frc.robot.command;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.beluga.arm.SUB_Arm;
import frc.robot.beluga.conveyor.SUB_Conveyor;

public class CMD_Intake extends Command{

  private final SUB_Conveyor intexer;
  private final SUB_Arm arm;

  private final Supplier<Boolean> manualCancel;

  private boolean isCommandDone = false;

  public CMD_Intake(SUB_Conveyor intexer, SUB_Arm arm, Supplier<Boolean> manualCancel){
    this.intexer = intexer;
    this.arm = arm;
    this.manualCancel = manualCancel;

    addRequirements(intexer, arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    DriverStation.reportWarning("[init] CMD_Intake running", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    // Set intexer to intake mode to initally pick up the donut
    intexer.setState(Constants.States.ConveyorState.INTAKE);

    // If intake sensor is triggers raise the arm to pick the donut up
    if(intexer.inputs.intakeInitalSensorState){
      arm.setState(Constants.States.ArmState.INTAKE);
    }
    // If the dount travels into the indexer and hits last sensor end the command
    if(intexer.inputs.indexerFinalSensorState){
      intexer.setState(Constants.States.ConveyorState.OFF);
      isCommandDone = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
    // Stop the intexer
    intexer.setState(Constants.States.ConveyorState.OFF);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished(){
    if(isCommandDone){
      return true;
    }
    if(manualCancel.get()){
      return true;
    }
    return false;
  }
}
