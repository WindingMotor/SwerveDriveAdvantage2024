// Written by WindingMotor as part of the wmlib2j library.

package frc.robot.commands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.States.ShooterMode;
import frc.robot.subsystems.arm.SUB_Arm;
import frc.robot.subsystems.conveyor.SUB_Conveyor;
import frc.robot.subsystems.shooter.SUB_Shooter;
import frc.robot.subsystems.vision.SUB_Vision;
import frc.robot.util.AddressableLedStrip;
import frc.robot.util.AddressableLedStrip.LEDState;
import java.util.function.Supplier;

/**
 * Class to handle shooting commands.
 */
public class CMD_Shoot extends Command{

    private final SUB_Conveyor conveyor;
    private final SUB_Arm arm;
    private final SUB_Shooter shooter;
    private final SUB_Vision vision;
    private final AddressableLedStrip led;
    private ShooterMode mode;
    private final Supplier<Boolean> manualCancel;
    private final Supplier<Boolean> shoot;
    private boolean isCommandDone = false;
    private boolean hasShootBeenCalled = false;
    private boolean autoShoot = false;

    /**
     * Constructor for the CMD_Shoot command.
     *
     * @param conveyor        The conveyor subsystem.
     * @param arm            The arm subsystem.
     * @param shooter        The shooter subsystem.
     * @param vision         The vision subsystem.
     * @param led            The addressable led strip.
     * @param mode           The shooting mode.
     * @param manualCancel   The supplier to determine if the command should be manually cancelled.
     */
    public CMD_Shoot(SUB_Conveyor conveyor, SUB_Arm arm, SUB_Shooter shooter, SUB_Vision vision, AddressableLedStrip led, ShooterMode mode, Supplier<Boolean> manualCancel, Supplier<Boolean> shoot){
        this.conveyor = conveyor;
        this.arm = arm;
        this.shooter = shooter;
        this.vision = vision;
        this.led = led;
        this.mode = mode;
        this.shoot = shoot;
        this.manualCancel = manualCancel;

        addRequirements(conveyor, arm, shooter);
    }

    // Auto Shoot Contructor
    public CMD_Shoot(SUB_Conveyor conveyor, SUB_Arm arm, SUB_Shooter shooter, SUB_Vision vision, AddressableLedStrip led, ShooterMode mode, Supplier<Boolean> manualCancel, Supplier<Boolean> shoot, boolean autoShoot){
        this.conveyor = conveyor;
        this.arm = arm;
        this.shooter = shooter;
        this.vision = vision;
        this.led = led;
        this.mode = mode;
        this.shoot = shoot;
        this.autoShoot = autoShoot;
        this.manualCancel = manualCancel;

        addRequirements(conveyor, arm, shooter);
    }

    /**
     * Reports to the driver station that the command is
     * running and reset all the flags.
    */
    @Override
    public void initialize(){
        isCommandDone = false;
        hasShootBeenCalled = false; 

        DriverStation.reportWarning("[init] CMD_Shoot Running with " + mode.toString() + " mode", false);
    }

    /**
     * Spool up the shooter to the correct rpm and set arm angle depending on the mode.
    */
    @Override
    public void execute(){
        setInitalStates();
        checkConveyor();
        checkEndCommand();
    }

    private void setInitalStates(){
        // SPEAKER mode
        if(mode == ShooterMode.SPEAKER){ 
            shooter.invertMotors(true);
            shooter.setState(Constants.States.ShooterState.SPEAKER_2M);
            arm.setState(Constants.States.ArmState.SPEAKER_2M);

        // AMP mode
        }else if(mode == ShooterMode.AMP){ 
            shooter.invertMotors(false);
            shooter.setState(Constants.States.ShooterState.AMP);
            arm.setState(Constants.States.ArmState.AMP);

        // Dynamic mode
        }else{ 
        
        }
    }

    private void checkConveyor(){

        // Manual operator shoot
        if(!autoShoot){
            if(shoot.get()){
                hasShootBeenCalled = true;
                if(mode == ShooterMode.AMP){
                    conveyor.setState(Constants.States.ConveyorState.AMP);
                }else{
                    conveyor.setState(Constants.States.ConveyorState.SHOOT);
                }
            }
            
        // Auto shoot with RPM check
        }else{ 
            if(shooter.inputs.isUpToSpeed){ 
                hasShootBeenCalled = true;
                if(mode == ShooterMode.AMP){
                    conveyor.setState(Constants.States.ConveyorState.AMP);
                }else{
                    conveyor.setState(Constants.States.ConveyorState.SHOOT);
                }
            }
        }
    }

    private void checkEndCommand(){

        // Once operator has pressed shoot button and the donut leaves the conveyor end the command
        if(hasShootBeenCalled){
            if(conveyor.inputs.indexerFinalSensorState){
                isCommandDone = true;
            }
        }
    }

    /**
     * Stops and idles the robots subsystems.
     * @param  interrupted   Indicates if the command was interrupted
     */
    @Override
    public void end(boolean interrupted){
        led.setState(LEDState.RAINBOW);
    }

    /**
     * Checks if the command is finished. Can be overriden by manual cancel.
     * @return  True if the task is finished, false otherwise
    */
    @Override
    public boolean isFinished(){
        if(manualCancel.get() || isCommandDone){
            return true;
        }else{
            return false;
        }
    }
}