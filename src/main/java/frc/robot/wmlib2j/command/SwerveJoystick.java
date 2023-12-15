package frc.robot.wmlib2j.command;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.wmlib2j.swerve.Swerve;
import java.util.function.Supplier;

public class SwerveJoystick extends Command{

    private final Supplier<Double> xInput;
    private final Supplier<Double> yInput;
    private final Supplier<Double> rInput;
    private final Supplier<Boolean> isFieldOriented;
    private final Swerve swerve;

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
        addRequirements(swerve);
    }

    @Override
    public void execute(){
        double xCurrent = xInput.get();
        double yCurrent = yInput.get();
        double rCurrent = rInput.get();

        ChassisSpeeds newSpeeds;
        if (isFieldOriented.get()){
            newSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xCurrent * Constants.Kinematics.MAX_LINEAR_VELOCITY,
                    yCurrent * Constants.Kinematics.MAX_LINEAR_VELOCITY,
                    rCurrent * Constants.Kinematics.MAX_ANGULAR_VELOCITY,
                    swerve.gyroInputs.yawPosition);
        }else{
            newSpeeds = new ChassisSpeeds(
                    xCurrent * Constants.Kinematics.MAX_LINEAR_VELOCITY,
                    yCurrent * Constants.Kinematics.MAX_LINEAR_VELOCITY,
                    rCurrent * Constants.Kinematics.MAX_ANGULAR_VELOCITY);
        }

        swerve.runWithSpeeds(newSpeeds);
    }

    @Override
    public void end(boolean interrupted){
        swerve.stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
