
package frc.robot.beluga.shooter;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.States.ShooterState;

public class SUB_Shooter extends SubsystemBase{

    private final IO_ShooterBase io;

    public final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

    private ShooterState lastState;
    private ShooterState state;
    private double setpointRPM;
    private boolean dynamicMode;

    public SUB_Shooter(IO_ShooterBase io){
        this.io = io;
        this.lastState = ShooterState.OFF;
        this.state = ShooterState.OFF;
        this.setpointRPM = 0.0;
        this.dynamicMode = false;
    }

    @Override
    public void periodic(){
        // Update the inputs
        io.updateInputs(inputs);

        // Process inputs and send to logger
        Logger.processInputs("Shooter", inputs);

        // Update the PID based off rpm
        if(!dynamicMode){
            setpointRPM = state.rpm;
        }
        io.updatePID(setpointRPM);
    }

    public void setState(ShooterState newState){
        if(newState != lastState){
            lastState = state;
            state = newState;
        }
        if(newState == ShooterState.DYNAMIC){
            setDynamicMode(true);
        }else{
            setDynamicMode(false);
        }
    }

    public ShooterState getState(){
        return state;
    }

    public ShooterState getLastState(){
        return lastState;
    }

    public void updateSetpointRPM(double newSetpointRPM){
        this.setpointRPM = newSetpointRPM; 
    }

    public void stop(){
        io.stop();
    }

    private void setDynamicMode(boolean dynamicMode){
        this.dynamicMode = dynamicMode;
    }

}