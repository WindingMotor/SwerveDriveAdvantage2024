// Written by WindingMotor, 2024, Crescendo

package frc.robot.auto;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.States.ShooterMode;
import frc.robot.commands.CMDGR_Shoot;
import frc.robot.commands.CMD_Eject;
import frc.robot.commands.CMD_Idle;
import frc.robot.commands.CMD_Intake;
import frc.robot.commands.CMD_Led;
import frc.robot.subsystems.arm.SUB_Arm;
import frc.robot.subsystems.conveyor.SUB_Conveyor;
import frc.robot.subsystems.shooter.SUB_Shooter;
import frc.robot.subsystems.swerve.SUB_Swerve;
import frc.robot.subsystems.vision.SUB_Vision;
import frc.robot.util.AddressableLedStrip;
import frc.robot.util.AddressableLedStrip.LEDState;

public class CommandRegistrar{
    
    private final SUB_Vision vision;
    
    private final SUB_Swerve swerve;

    private final SUB_Conveyor conveyor;

    private final SUB_Arm arm;

    private final SUB_Shooter shooter;

    private final AddressableLedStrip led;

    public CommandRegistrar(SUB_Vision vision, SUB_Swerve swerve, SUB_Conveyor conveyor, SUB_Arm arm, SUB_Shooter shooter, AddressableLedStrip led){ 
        this.vision = vision;
        this.swerve = swerve;
        this.conveyor = conveyor;
        this.arm = arm;
        this.shooter = shooter;
        this.led = led;
    }

    /* 
     * Register all commands for autonomous to be called with pathplanner. 
     * Any manual disabling of commands has been turned off by default.
    */
    public void register(){

        // Intake command
        NamedCommands.registerCommand("Intake", new CMD_Intake(conveyor, arm,  () -> false));

        // Eject command
        NamedCommands.registerCommand("Eject", new CMD_Eject(conveyor, arm, () -> false));
        
        // Speaker shoot command, with auto shoot 
        NamedCommands.registerCommand("Shoot_Speaker", new CMDGR_Shoot(conveyor, arm, shooter, vision, led, ShooterMode.SPEAKER, () -> false, () -> false, true));

        // Amp shoot command, with auto shoot 
        NamedCommands.registerCommand("Shoot_Amp", new CMDGR_Shoot(conveyor, arm, shooter, vision, led, ShooterMode.AMP, () -> false, () -> false, true));

        // Idle command
        NamedCommands.registerCommand("Idle", new CMD_Idle(conveyor, arm, shooter));

        // LED commands
        NamedCommands.registerCommand("Led_Rainbow", new CMD_Led(led, LEDState.RAINBOW));
        NamedCommands.registerCommand("Led_Green", new CMD_Led(led, LEDState.GREEN));
        NamedCommands.registerCommand("Led_Red", new CMD_Led(led, LEDState.RED));
        NamedCommands.registerCommand("Led_Blue", new CMD_Led(led, LEDState.BLUE));
    }
}
