// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.beluga.shooter;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.wmlib2j.vision.Vision;

/** An example command that uses an example subsystem. */
public class Shoot extends Command {

  private final Shooter shooter;
  private final Vision vision;

  private final Supplier<Boolean> manualCancel;

  public Shoot(Shooter shooter, Vision vision, Supplier<Boolean> manualCancel){
    this.shooter = shooter;
    this.vision = vision;
    this.manualCancel = manualCancel;

    addRequirements(shooter, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
    shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished(){
    if(manualCancel.get()){
      return true;
    }
    return false;
  }
}
