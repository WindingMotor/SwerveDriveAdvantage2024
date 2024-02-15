// Written by WindingMotor as part of the wmlib2j library.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class CMD_ButtonCancel extends Command{


    private final Supplier<Boolean> cancel;
    private boolean isCommandDone = false;

    public CMD_ButtonCancel(Supplier<Boolean> cancel){
        this.cancel = cancel;
    }

    @Override
    public void initialize(){
        isCommandDone = false;
    }

    @Override
    public void execute(){
        if(cancel.get()){
            isCommandDone = true;
        }
    }

    @Override
    public boolean isFinished(){
        if(isCommandDone){
            return true;
        }else{
            return false;  
        }
    }

}

