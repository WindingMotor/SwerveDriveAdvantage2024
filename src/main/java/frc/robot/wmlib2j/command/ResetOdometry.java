
package frc.robot.wmlib2j.command;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.wmlib2j.swerve.Swerve;
import java.util.function.Supplier;

/**
 * Controls the swerve drive with joystick axes.
 * Creates chassis speeds and sends them to the swerve subsystem.
*/
public class ResetOdometry extends Command{

    // The swerve drive subsystem
    private final Swerve swerve;

    public ResetOdometry(Swerve swerve){

        this.swerve = swerve;
        // This command requires the swerve drive subsystem.
        addRequirements(swerve);
    }


    @Override
    public void initialize(){
        
    }

    /**
     * When commands ends tell swerve subsystem to stop modules. 
    */
    @Override
    public void end(boolean interrupted){
        // Stop the swerve drive when the command ends
        swerve.stop();
    }

    /**
     * Never end the command as it should run forever.
    */
    @Override
    public boolean isFinished(){
        return false;
    }

}