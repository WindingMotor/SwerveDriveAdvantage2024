
package frc.robot.command;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.States.ShooterMode;
import frc.robot.beluga.arm.SUB_Arm;
import frc.robot.beluga.conveyor.SUB_Conveyor;
import frc.robot.beluga.shooter.SUB_Shooter;
import frc.robot.wmlib2j.util.MathCalc;
import frc.robot.wmlib2j.vision.SUB_Vision;

public class CMD_Shoot extends Command{

  private final SUB_Conveyor intexer;
  private final SUB_Arm arm;
  private final SUB_Shooter shooter;
  private final SUB_Vision vision;
  private ShooterMode mode;

  private final Supplier<Boolean> manualCancel;
  

  private boolean isCommandDone = false;

  public CMD_Shoot(SUB_Conveyor intexer, SUB_Arm arm, SUB_Shooter shooter, SUB_Vision vision, ShooterMode mode, Supplier<Boolean> manualCancel){
    this.intexer = intexer;
    this.arm = arm;
    this.shooter = shooter;
    this.vision = vision;
    this.mode = mode;
    this.manualCancel = manualCancel;

    addRequirements(intexer, arm, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    DriverStation.reportWarning("[init] CMD_Shoot Running with " + mode.toString() + " mode", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){

    // Spool up the shooter to the correct rpm and set arm angle depending on the mode
    if(mode == ShooterMode.SPEAKER){ // SPEAKER mode
      shooter.setState(Constants.States.ShooterState.SPEAKER);
      arm.setState(Constants.States.ArmState.SPEAKER);

    }else if(mode == ShooterMode.AMP){ // AMP mode
      shooter.setState(Constants.States.ShooterState.AMP);
      arm.setState(Constants.States.ArmState.AMP);

    }else{ // DYNAMIC mode, automatically calculates and sets the rpm and angle 

      arm.setState(Constants.States.ArmState.IDLE);
      
      double calculatedAngle = 45.0;
      arm.updateSetpointAngle(calculatedAngle);

      shooter.setState(Constants.States.ShooterState.SPEAKER);

      double targetHorizontalDistance = vision.getDirectDistanceMetersToTag(3);
      double calculatedRPM = MathCalc.calculateRPM(calculatedAngle, targetHorizontalDistance);

      Logger.recordOutput( "CMD_Shoot Calculated RPM", calculatedRPM);
      Logger.recordOutput( "CMD_Shoot Target Distance", calculatedRPM);

      shooter.updateSetpointRPM(calculatedRPM);
    }

    // When the shooter and arm hit their required setpoint run the intexer
    if(shooter.inputs.isUpToSpeed && arm.inputs.isAtSetpoint){
      intexer.setState(Constants.States.ConveyorState.SHOOT);
    }
    
    // When the donut leaves the indexer inital sensor and also leaves the final sensor end the command
    if(!intexer.inputs.indexerInitalSensorState && !intexer.inputs.indexerFinalSensorState){
        isCommandDone = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
    // Stop or idle the robot subsystems
    intexer.setState(Constants.States.ConveyorState.OFF);
    arm.setState(Constants.States.ArmState.IDLE);
    shooter.setState(Constants.States.ShooterState.IDLE);
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
