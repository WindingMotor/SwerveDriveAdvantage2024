package frc.robot.wmlib2.swerve;

import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.LogTable;

// The base interface for using the swerve module abstracts to the real and simulation classes.
public interface IO_ModuleBase{

    class ModuleInputs implements LoggableInputs{

        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedPercentage = 0.0;
        public double driveCurrentAmps = 0.0;
        public double driveTempCelsius = 0.0;

        public double turnAbsolutePositionRad = 0.0;
        public double turnPositionRad = 0.0;
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedPercentage = 0.0;
        public double turnCurrentAmps = 0.0;
        public double turnTempCelsius = 0.0;

        @Override
        public void toLog(LogTable table){
            table.put("drivePositionRad", drivePositionRad);
            table.put("driveVelocityRadPerSec", driveVelocityRadPerSec);
            table.put("driveAppliedPercentage", driveAppliedPercentage);
            table.put("driveCurrentAmps", driveCurrentAmps);
            table.put("driveTempCelsius", driveTempCelsius);

            table.put("turnAbsolutePositionRad", turnAbsolutePositionRad);
            table.put("turnPositionRad", turnPositionRad);
            table.put("turnVelocityRadPerSec", turnVelocityRadPerSec);
            table.put("turnAppliedPercentage", turnAppliedPercentage);
            table.put("turnCurrentAmps", turnCurrentAmps);
            table.put("turnTempCelsius", turnTempCelsius);
        }

        @Override
        public void fromLog(LogTable table){
            // Drive
            drivePositionRad = table.get("drivePositionRad", drivePositionRad);
            driveVelocityRadPerSec = table.get("driveVelocityRadPerSec", driveVelocityRadPerSec);
            driveAppliedPercentage = table.get("driveAppliedPercentage", driveAppliedPercentage);
            driveCurrentAmps = table.get("driveCurrentAmps", driveCurrentAmps);
            driveTempCelsius = table.get("turnTempCelsius", driveTempCelsius);

            // Turn 
            turnAbsolutePositionRad = table.get("turnAbsolutePositionRad", turnAbsolutePositionRad);
            turnPositionRad = table.get("turnPositionRad", turnPositionRad);
            turnVelocityRadPerSec = table.get("turnVelocityRadPerSec", turnVelocityRadPerSec);
            turnAppliedPercentage = table.get("turnAppliedPercentage", turnAppliedPercentage);
            driveCurrentAmps = table.get("driveCurrentAmps", driveCurrentAmps);
            turnTempCelsius = table.get("turnTempCelsius", turnTempCelsius);
        }
    }

    void updateInputs(ModuleInputs inputs);

    void setDriveOutput(double percent);

    void setTurnOutput(double percent);

    void stop();

    void setPIDReferences(double driveReference, double turnReference);

}
