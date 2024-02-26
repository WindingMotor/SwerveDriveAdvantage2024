// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.States.ShooterMode;
import frc.robot.auto.AutoSelector;
import frc.robot.auto.CommandRegistrar;
import frc.robot.commands.CMDGR_DrivePose;
import frc.robot.commands.CMDGR_Shoot;
import frc.robot.commands.CMD_Climb;
import frc.robot.commands.CMD_Eject;
import frc.robot.commands.CMD_Idle;
import frc.robot.commands.CMD_Intake;
import frc.robot.subsystems.arm.IO_ArmReal;
import frc.robot.subsystems.arm.SUB_Arm;
import frc.robot.subsystems.climb.IO_ClimbReal;
import frc.robot.subsystems.climb.SUB_Climb;
import frc.robot.subsystems.conveyor.IO_ConveyorReal;
import frc.robot.subsystems.conveyor.SUB_Conveyor;
import frc.robot.subsystems.shooter.IO_ShooterReal;
import frc.robot.subsystems.shooter.SUB_Shooter;
import frc.robot.subsystems.swerve.IO_SwerveReal;
import frc.robot.subsystems.swerve.SUB_Swerve;
import frc.robot.subsystems.vision.IO_VisionReal;
import frc.robot.subsystems.vision.SUB_Vision;
import frc.robot.util.AddressableLedStrip;

public class RobotContainer {

  private final CommandXboxController driverController = new CommandXboxController(3);
  private final CommandXboxController operatorController = new CommandXboxController(4);

  private final SUB_Vision vision = new SUB_Vision(new IO_VisionReal());

  // All methods using these subsystems should be called in this order -> conveyor, arm, shooter
  private final SUB_Conveyor conveyor = new SUB_Conveyor(new IO_ConveyorReal());

  private final SUB_Arm arm = new SUB_Arm(new IO_ArmReal());

  private final SUB_Shooter shooter = new SUB_Shooter(new IO_ShooterReal());

  private final AddressableLedStrip led = new AddressableLedStrip(0, 71);

  private final SUB_Swerve swerve = new SUB_Swerve(new IO_SwerveReal(), vision);

  private final SUB_Climb climb = new SUB_Climb(new IO_ClimbReal());

  private final CommandRegistrar commandRegistrar =
      new CommandRegistrar(vision, swerve, conveyor, arm, shooter, led);

  private final AutoSelector autoSelector;

  public RobotContainer() {

    commandRegistrar.register();

    autoSelector = new AutoSelector();

    configureBindings();

    logMetadata();
  }

  public void configDriving() {

    var alliance = DriverStation.getAlliance();

    // Get the pose for the correct alliance and set the pose variables
    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Red) {

        // Default drive command
        swerve.setDefaultCommand(
            swerve.driveJoystick(
                () -> -driverController.getRawAxis(1),
                () -> driverController.getRawAxis(0),
                () -> -driverController.getRawAxis(3)));

      } else if (alliance.get() == Alliance.Blue) {

        // Default drive command
        swerve.setDefaultCommand(
            swerve.driveJoystick(
                () -> driverController.getRawAxis(1),
                () -> -driverController.getRawAxis(0),
                () -> -driverController.getRawAxis(3)));
      }
    }
  }
  /** Configure the bindings for the controller buttons to specific commands. */
  private void configureBindings() {

    // Shoot command
    operatorController
        .x()
        .onTrue(
            new CMDGR_Shoot(
                conveyor,
                arm,
                shooter,
                vision,
                led,
                ShooterMode.SPEAKER,
                () -> operatorController.b().getAsBoolean(),
                () -> operatorController.x().getAsBoolean()));

    // Amp command
    operatorController
        .y()
        .onTrue(
            new CMDGR_Shoot(
                conveyor,
                arm,
                shooter,
                vision,
                led,
                ShooterMode.AMP,
                () -> operatorController.b().getAsBoolean(),
                () -> operatorController.y().getAsBoolean()));

    // Trap command
    operatorController
        .rightStick()
        .onTrue(
            new CMDGR_Shoot(
                conveyor,
                arm,
                shooter,
                vision,
                led,
                ShooterMode.TRAP,
                () -> operatorController.b().getAsBoolean(),
                () -> operatorController.rightStick().getAsBoolean()));

    // Intake command
    operatorController
        .a()
        .onTrue(new CMD_Intake(conveyor, arm, () -> operatorController.b().getAsBoolean()));

    // Eject command
    operatorController
        .rightBumper()
        .onTrue(new CMD_Eject(conveyor, arm, () -> operatorController.b().getAsBoolean()));

    // Idle command
    operatorController.b().onTrue(new CMD_Idle(conveyor, arm, shooter));

    // Drive to speaker command
    operatorController
        .leftStick()
        .debounce(0.1)
        .onTrue(
            new CMDGR_DrivePose(
                swerve,
                Constants.Auto.DriveScoringPoseState.SPEAKER,
                () -> operatorController.b().getAsBoolean()));

    operatorController.leftBumper().onTrue(new CMD_Climb(led, climb, () -> 0.1));
    operatorController.leftBumper().onFalse(new CMD_Climb(led, climb, () -> 0.0));

    // operatorController.rightTrigger(0.1).onTrue(new CMD_Climb(led, climb, () -> -0.1));
    // Drive to amp command
    // operatorController.povLeft().debounce(0.25).onTrue(new CMDGR_DrivePose(swerve,
    // Constants.Auto.DriveScoringPoseState.AMP, () -> operatorController.b().getAsBoolean()));
  }

  private void logMetadata() {}

  // TEST INSERT

  /**
   * Get the autonomous command.
   *
   * @return The autonomous command
   */
  public Command getAutonomousCommand() {
    return autoSelector.getSelectedAuto();
  }
}
