
package frc.robot.beluga.shooter;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{

    private final IO_ShooterBase io;

    private final IO_ShooterBase.ShooterInputs inputs = new IO_ShooterBase.ShooterInputs();



    public Shooter(IO_ShooterBase io){
        this.io = io;
    }

    public void periodic(){


        // Update the inputs.
        io.updateInputs(inputs);

        // Process inputs and send to logger.
        Logger.processInputs("Shooter", inputs);

    }

    public void updateSetpoint(double setpointRPM){
        io.updateSetpoint(setpointRPM);
    }

    public void stop(){
        io.stop();
    }

    public double calculateRequiredRPM(){
        return 0.0;
    }

    public boolean getIsUpToSpeed(){
        return io.isUpToSpeed();
    }

    public boolean getBackLimitSwitchStatus(){
        return inputs.backLimitSwitchStatus;
    }


}