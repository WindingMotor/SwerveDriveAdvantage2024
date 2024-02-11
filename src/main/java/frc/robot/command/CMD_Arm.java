
package frc.robot.command;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.States.ArmState;
import frc.robot.beluga.arm.SUB_Arm;
import frc.robot.beluga.conveyor.SUB_Conveyor;
import frc.robot.beluga.shooter.SUB_Shooter;

public class CMD_Arm extends Command{

  private SUB_Arm arm;
  private ArmState state;

  public CMD_Arm(SUB_Arm arm, ArmState state){
    this.arm = arm;
    this.state = state;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    DriverStation.reportWarning("[init] CMD_Arm set new state", false);
    // Stop and idle the robot subsystems
    arm.setState(state);
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