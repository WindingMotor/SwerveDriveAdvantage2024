
package frc.robot.wmlib2j.command;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.wmlib2j.swerve.Swerve;
import java.util.function.Supplier;

// This class represents a joystick command for a swerve drive robot.
public class SwerveJoystick extends Command{

    // Input suppliers for joystick axes and field orientation mode.
    private final Supplier<Double> xInput;
    private final Supplier<Double> yInput;
    private final Supplier<Double> rInput;
    private final Supplier<Boolean> isFieldOriented;

    // The swerve drive subsystem.
    private final Swerve swerve;

    // Constructor for the SwerveJoystick command.
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

    // Executes the command.
    @Override
    public void execute(){
        // Get the current joystick inputs.
        double xCurrent = xInput.get();
        double yCurrent = yInput.get();
        double rCurrent = rInput.get();

        // Calculate the new speeds for the swerve drive.
        ChassisSpeeds newSpeeds;
        if (isFieldOriented.get()){
            // If field-oriented mode is enabled, calculate field-relative speeds.
            newSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xCurrent * Constants.Kinematics.MAX_LINEAR_VELOCITY,
                    yCurrent * Constants.Kinematics.MAX_LINEAR_VELOCITY,
                    rCurrent * Constants.Kinematics.MAX_ANGULAR_VELOCITY,
                    swerve.gyroInputs.yawPosition);
        }else{
            // If field-oriented mode is disabled, calculate robot-relative speeds.
            newSpeeds = new ChassisSpeeds(
                    xCurrent * Constants.Kinematics.MAX_LINEAR_VELOCITY,
                    yCurrent * Constants.Kinematics.MAX_LINEAR_VELOCITY,
                    rCurrent * Constants.Kinematics.MAX_ANGULAR_VELOCITY);
        }

        // Run the swerve drive with the new speeds.
        swerve.runWithSpeeds(newSpeeds);
    }

    // Ends the command.
    @Override
    public void end(boolean interrupted){
        // Stop the swerve drive when the command ends.
        swerve.stop();
    }

    // Determines whether the command is finished.
    @Override
    public boolean isFinished(){
        // This command never finishes on its own.
        return false;
    }
}