// Written by WindingMotor, 2024, Crescendo

package frc.robot.subsystems.shooter;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.States.ShooterState;

public class SUB_Shooter extends SubsystemBase{

    private final IO_ShooterBase io;

    public final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

    private ShooterState state;

    public SUB_Shooter(IO_ShooterBase io){
        this.io = io;
        this.state = ShooterState.OFF;
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        io.updatePID(state.rpm);
    }

    public void setState(ShooterState newState){
        state = newState;
    }

    public ShooterState getState(){
        return state;
    }

    public void stop(){
        io.stop();
    }

    public void invertMotors(boolean inverted){
        io.invertMotors(inverted);
    }

}