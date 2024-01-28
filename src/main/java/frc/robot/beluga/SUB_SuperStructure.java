
package frc.robot.beluga;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.States.SuperStructureState;
import frc.robot.beluga.arm.SUB_Arm;
import frc.robot.beluga.shooter.SUB_Shooter;

public class SUB_SuperStructure extends SubsystemBase{

    private final SUB_Arm arm;
    private final SUB_Shooter shooter;

    private SuperStructureState lastState;
    private SuperStructureState state;

    public SUB_SuperStructure(SUB_Arm arm, SUB_Shooter shooter){
        this.arm = arm;
        this.shooter = shooter;
        state = SuperStructureState.OFF;
        lastState = SuperStructureState.OFF;
    }

    public void periodic(){
        arm.setState(state.armState);
        shooter.setState(state.shooterState);
    }

    public Command setState(SuperStructureState newState){
        return run(()->{
            if(newState != lastState){
                lastState = state;
                state = newState;
            }
        });
    }
}
