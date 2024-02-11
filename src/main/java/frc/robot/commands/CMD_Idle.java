
package frc.robot.commands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.arm.SUB_Arm;
import frc.robot.subsystems.conveyor.SUB_Conveyor;
import frc.robot.subsystems.shooter.SUB_Shooter;

public class CMD_Idle extends Command{

  private final SUB_Conveyor conveyor;
  private final SUB_Arm arm;
  private final SUB_Shooter shooter;

  public CMD_Idle(SUB_Conveyor conveyor, SUB_Arm arm, SUB_Shooter shooter){
    this.conveyor = conveyor;
    this.arm = arm;
    this.shooter = shooter;
    addRequirements(conveyor, arm, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    DriverStation.reportWarning("[init] CMD_Idle running", false);
    // Stop and idle the robot subsystems
    conveyor.setState(Constants.States.ConveyorState.OFF);
    arm.setState(Constants.States.ArmState.IDLE);
    shooter.setState(Constants.States.ShooterState.IDLE);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){}

  // Returns true when the command should end.
  @Override
  public boolean isFinished(){ return true;}

}