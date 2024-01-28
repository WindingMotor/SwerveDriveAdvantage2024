
package frc.robot.beluga;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.States.SuperStructureStatePosition;
import frc.robot.beluga.arm.SUB_Arm;
import frc.robot.beluga.shooter.SUB_Shooter;

public class SUB_SuperStructure extends SubsystemBase{

    private final SUB_Arm arm;
    private final SUB_Shooter shooter;

    private SuperStructureStatePosition lastState;
    private SuperStructureStatePosition state;

    public SUB_SuperStructure(SUB_Arm arm, SUB_Shooter shooter){
        this.arm = arm;
        this.shooter = shooter;
        state = SuperStructureStatePosition.OFF;
        lastState = SuperStructureStatePosition.OFF;
    }

    public void periodic(){
        arm.setState(state.armState);
        shooter.setState(state.shooterState);

        // If the robot is in intake mode and the game piece is picked up by the intake set the robot to idle mode.
        if(state == SuperStructureStatePosition.INTAKE){
            // intake.getFirstSensor()
            if(true){
                setStatePosition(SuperStructureStatePosition.IDLE);
            }
        }

        // 
    }

    public Command setStatePosition(SuperStructureStatePosition newState){
        return run(()->{
            if(newState != lastState){
                lastState = state;
                state = newState;
            }
        });
    }

    public void setActiveState(){}
}
