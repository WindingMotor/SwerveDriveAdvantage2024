
package frc.robot.beluga.shooter;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{

    private final IO_ShooterBase io;

    private final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

    public Shooter(IO_ShooterBase io){
        this.io = io;
    }

    public void periodic(){


        // Update the inputs.
        io.updateInputs(inputs);

        // Process inputs and send to logger.
        Logger.processInputs("Shooter", inputs);

    }

    public void stop(){
        io.stop();
    }

}