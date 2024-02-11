package frc.robot.command.shoot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.States.ConveyorState;
import frc.robot.Constants.States.ShooterMode;
import frc.robot.beluga.arm.SUB_Arm;
import frc.robot.beluga.conveyor.SUB_Conveyor;
import frc.robot.beluga.shooter.SUB_Shooter;
import frc.robot.wmlib2j.util.AddressableLedStrip;
import frc.robot.wmlib2j.util.AddressableLedStrip.LEDState;
import frc.robot.wmlib2j.vision.SUB_Vision;

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

    /**
     * Reports to the driver station that the command is running.
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
        if(mode == ShooterMode.SPEAKER){ // SPEAKER mode
            shooter.invertMotors(true);

            //shooter.setState(Constants.States.ShooterState.SPEAKER_1M);
            //arm.setState(Constants.States.ArmState.SPEAKER_1M);

        }else if(mode == ShooterMode.AMP){ // AMP mode
            shooter.invertMotors(false);
            shooter.setState(Constants.States.ShooterState.AMP);
            arm.setState(Constants.States.ArmState.AMP);
        }else{ dynamicMode(); } // DYNAMIC mode

        // If operator presses shoot button run the conveyor to shoot
        if(shoot.get()){
            hasShootBeenCalled = true;
            if(mode == ShooterMode.AMP){
                conveyor.setState(Constants.States.ConveyorState.AMP);
            }else{
                conveyor.setState(Constants.States.ConveyorState.SHOOT);
            }
        }

        /*
        // Once operator has pressed shoot button and the donut leaves the conveyor end the command
        if(hasShootBeenCalled){
            if(!conveyor.inputs.indexerFinalSensorState){
                isCommandDone = true;
                
            }
        }
         */

    }

    /**
     * Stops and idles the robots subsystems.
     * @param  interrupted   Indicates if the command was interrupted
     */
    @Override
    public void end(boolean interrupted){
        conveyor.setState(Constants.States.ConveyorState.OFF);
        arm.setState(Constants.States.ArmState.IDLE);
        shooter.setState(Constants.States.ShooterState.IDLE);
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

    /**
     * Handles dynamic mode. Sets the state of the arm to IDLE,
     * calculates the angle of the arm, and sets the angle and rpm of the shooter.
    */
    private void dynamicMode(){


        shooter.invertMotors(true);
        
        // Set state of arm to IDLE
        arm.setState(Constants.States.ArmState.IDLE);

        // Calculate the angle of the arm
        double calculatedAngle = 0.0;

        // Set the angle of the arm
        // TODO: Set the angle of the arm

        // Set the rpm of the shooter
        // TODO: Set the rpm of the shooter
    }
}