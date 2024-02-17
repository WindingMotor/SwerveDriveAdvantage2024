// Written by WindingMotor, 2024, Crescendo
package frc.robot.subsystems.arm;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.States.ArmState;

public class SUB_Arm extends SubsystemBase{

    private final IO_ArmBase io;

    public final ArmInputsAutoLogged inputs = new ArmInputsAutoLogged();
    
    private ArmState state;

    public SUB_Arm(IO_ArmBase io){
        this.io = io;
        state = ArmState.OFF;;
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
        io.updatePID(state.position);
    }

    public void setState(ArmState newState){
        state = newState;
    }

    public ArmState getState(){
        return state;
    }

    public void stop(){
        io.stop();
    }

    public boolean isAtSetpoint(){
        return inputs.isAtSetpoint;
    }

}