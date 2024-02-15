
package frc.robot.commands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants;
import frc.robot.Constants.Auto.DriveScoringPoseState;
import frc.robot.subsystems.swerve.SUB_Swerve;

import java.util.function.Supplier;

public class CMDGR_DrivePose extends ParallelRaceGroup{
    
    public CMDGR_DrivePose(SUB_Swerve swerve, DriveScoringPoseState drivePoseState, Supplier<Boolean> cancel){

        if(Constants.TELEOP_AUTO_DRIVE_ENABLED){            
            var alliance = DriverStation.getAlliance();
            Pose2d speakerPose = new Pose2d();
            Pose2d ampPose = new Pose2d();

            // Get the pose for the correct alliance and set the pose variables
            if(alliance.isPresent()){
                if(alliance.get() == Alliance.Red){
                    speakerPose = Constants.Auto.ScoringPoses.RED_SPEAKER.pose;
                    ampPose = Constants.Auto.ScoringPoses.RED_AMP.pose;
                }else if(alliance.get() == Alliance.Blue){
                    speakerPose = Constants.Auto.ScoringPoses.BLU_SPEAKER.pose;
                    ampPose = Constants.Auto.ScoringPoses.BLU_AMP.pose;
                }
            }

            // Get the drive command depending on the drivePoseState
            Command driveCommand;
            if(drivePoseState == DriveScoringPoseState.SPEAKER){
                driveCommand = swerve.driveToPose(speakerPose);
            }else if(drivePoseState == DriveScoringPoseState.AMP){
                driveCommand = swerve.driveToPose(ampPose);
            }else{
                driveCommand = new PrintCommand("[error] Unknown drivePoseState: " + drivePoseState);
            }
            
            // Add the commands
            addCommands(
                driveCommand,
                new CMD_ButtonCancel(cancel)
            );
        }else{
            addCommands(new PrintCommand("[error] TELEOP_AUTO_DRIVE_ENABLED is disabled"));
        }
    }
    

}