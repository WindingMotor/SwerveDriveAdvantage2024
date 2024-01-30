
package frc.robot.beluga.shooter;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.States.ShooterState;

public class SUB_Shooter extends SubsystemBase{

    private final IO_ShooterBase io;

    public final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

    private ShooterState lastState;
    private ShooterState state;
    private double rpm;

    public SUB_Shooter(IO_ShooterBase io){
        this.io = io;
        this.lastState = ShooterState.OFF;
        this.state = ShooterState.OFF;
        this.rpm = 0.0;
    }

    @Override
    public void periodic(){
        // Update the inputs
        io.updateInputs(inputs);

        // Process inputs and send to logger
        Logger.processInputs("Shooter", inputs);

        // Update the PID based off rpm
        rpm = state.rpm;
        io.updatePID(rpm);
    }

    public void setState(ShooterState newState){
        if(newState != lastState){
            lastState = state;
            state = newState;
        }
    }

    public ShooterState getState(){
        return state;
    }

    public ShooterState getLastState(){
        return lastState;
    }

    public void setRPM(double rpm){
        this.rpm = rpm; 
    }

    public void stop(){
        io.stop();
    }

}