package frc.robot.wmlib2j.swerve;

import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.LogTable;

/**
 * The base interface for using the swerve module, abstracts to the real and simulation classes.
 * This interface provides methods for updating inputs, setting drive and turn outputs, stopping the module,
 * and setting PID references for the drive and turn motors.
*/
public interface IO_ModuleBase{

    /**
     * Represents the inputs for a module.
     * It contains fields for drive motor values and turn motor values.
    */
    class ModuleInputs implements LoggableInputs{

        // Drive motor values
        public double drivePositionMeters = 0.0;
        public double driveVelocityMetersPerSec = 0.0;
        public double driveAppliedPercentage = 0.0;
        public double driveCurrentAmps = 0.0;
        public double driveTempCelsius = 0.0;

        // Turn motor values
        public double turnAbsolutePositionRad = 0.0;
        public double turnPositionRad = 0.0;
        public double turnVelocityRadPerSec = 0.0;
        public double turnAppliedPercentage = 0.0;
        public double turnCurrentAmps = 0.0;
        public double turnTempCelsius = 0.0;

        @Override
        /**
         * Converts the current state of the module inputs into a loggable format.
         * @param table The log table to write the inputs to.
        */
        public void toLog(LogTable table){
            table.put("drivePositionMeters", drivePositionMeters);
            table.put("driveVelocityMetersPerSec", driveVelocityMetersPerSec);
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
        /**
         * Reads the state of the module inputs from a log table.
         * @param table The log table to read the inputs from.
        */
        public void fromLog(LogTable table){
            // Drive
            drivePositionMeters = table.get("drivePositionMeters", drivePositionMeters);
            driveVelocityMetersPerSec = table.get("driveVelocityMetersPerSec", driveVelocityMetersPerSec);
            driveAppliedPercentage = table.get("driveAppliedPercentage", driveAppliedPercentage);
            driveCurrentAmps = table.get("driveCurrentAmps", driveCurrentAmps);
            driveTempCelsius = table.get("turnTempCelsius", driveTempCelsius);

            // Turn inputs
            turnAbsolutePositionRad = table.get("turnAbsolutePositionRad", turnAbsolutePositionRad);
            turnPositionRad = table.get("turnPositionRad", turnPositionRad);
            turnVelocityRadPerSec = table.get("turnVelocityRadPerSec", turnVelocityRadPerSec);
            turnAppliedPercentage = table.get("turnAppliedPercentage", turnAppliedPercentage);
            driveCurrentAmps = table.get("driveCurrentAmps", driveCurrentAmps);
            turnTempCelsius = table.get("turnTempCelsius", turnTempCelsius);
        }
    }

    /**
     * Updates the inputs with the current values.
     * @param inputs The inputs to update.
    */
    void updateInputs(ModuleInputs inputs);

    /**
     * [EMPTY] Sets the drive motor speed.
     * @param percent The percentage of the drive output to set from -1.0 to 0.0.
    */
    void setDriveOutput(double percent);

    /**
     * [EMPTY] Sets the turn motor speed.
     * @param percent The percentage of the turn output to set from -1.0 to 0.0.
    */
    void setTurnOutput(double percent);

    /**
     * [EMPTY] Stops both the drive and turn motors.
    */
    void stop();

    /**
     * [EMPTY] Sets the SparkMax PID references. The main way for controlling the motors.
     * @param driveReference The drive reference in meters per sec.
     * @param turnReference The turn reference in radians.
    */
    void setPIDReferences(double driveReference, double turnReference);

}
