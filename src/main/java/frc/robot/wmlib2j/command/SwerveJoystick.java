// Written by WindingMotor as part of the wmlib2j library.

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
public class SwerveJoystick extends Command{

    // Input suppliers for joystick axes and field orientation mode
    private final Supplier<Double> xInput;
    private final Supplier<Double> yInput;
    private final Supplier<Double> rInput;
    private final Supplier<Boolean> isFieldOriented;

    // The swerve drive subsystem
    private final Swerve swerve;

    /**
     * Constructor for the SwerveJoystick command.
     * @param xInput The x-axis input from the joystick.
     * @param yInput The y-axis input from the joystick.
     * @param rInput The rotation input from the joystick.
     * @param isFieldOriented A boolean supplier indicating whether the robot is in field-oriented mode.
     * @param swerve The swerve drive subsystem.
    */
    public SwerveJoystick(
            Supplier<Double> xInput,
            Supplier<Double> yInput,
            Supplier<Double> rInput,
            Supplier<Boolean> isFieldOriented,
            Swerve swerve){

        this.xInput = xInput;
        this.yInput = yInput;
        this.rInput = rInput;
        this.isFieldOriented = isFieldOriented;
        this.swerve = swerve;

        // This command requires the swerve drive subsystem.
        addRequirements(swerve);
    }

    /**
     * Gets the current joystick inputs and calculates and sends new chassis speeds accordingly. 
    */
    @Override
    public void execute(){
        double x = xInput.get();
        double y = yInput.get();
        double r = rInput.get();

        ChassisSpeeds newSpeeds;
        if(isFieldOriented.get()){
            newSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    x * Constants.Kinematics.MAX_LINEAR_VELOCITY,
                    y * Constants.Kinematics.MAX_LINEAR_VELOCITY,
                    r * Constants.Kinematics.MAX_ANGULAR_VELOCITY,
                    swerve.gyroInputs.yawPosition
                ); 
        }else{
            newSpeeds = new ChassisSpeeds(
                    x * Constants.Kinematics.MAX_LINEAR_VELOCITY,
                    y * Constants.Kinematics.MAX_LINEAR_VELOCITY,
                    r * Constants.Kinematics.MAX_ANGULAR_VELOCITY
                );
        }

        swerve.updateSpeedSetpoint(newSpeeds);
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