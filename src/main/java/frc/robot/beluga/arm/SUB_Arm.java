
package frc.robot.beluga.arm;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.States.ArmState;

public class SUB_Arm extends SubsystemBase{

    private final IO_ArmBase io;

    public final ArmInputsAutoLogged inputs = new ArmInputsAutoLogged();

    private ArmState lastState;
    private ArmState state;

    private boolean dynamicMode;

    private double setpointAngle;

    public SUB_Arm(IO_ArmBase io){
        this.io = io;
        state = ArmState.OFF;
        lastState = ArmState.OFF;
        dynamicMode = false;
    }

    @Override
    public void periodic(){
        // Update the inputs
        io.updateInputs(inputs);

        // Process inputs and send to logger
        Logger.processInputs("Arm", inputs);
        
        if(!dynamicMode){
            setpointAngle = state.position;
        }
        // Update the PID
        io.updatePID(setpointAngle);
    }

    public void setState(ArmState newState){
        if(newState != lastState){
            lastState = state;
            state = newState;
        }
        if(newState == ArmState.DYNAMIC){
            setDynamicMode(true);
        }else{
            setDynamicMode(false);
        }
    }

    public ArmState getState(){
        return state;
    }

    public ArmState getLastState(){
        return lastState;
    }

    public void stop(){
        io.stop();
    }

    public void updateSetpointAngle(double angle){
        setpointAngle = angle;
    }

    public void setDynamicMode(boolean mode){
        dynamicMode = mode;
    }

}