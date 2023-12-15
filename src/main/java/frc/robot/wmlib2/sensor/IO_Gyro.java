package frc.robot.wmlib2.sensor;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface IO_Gyro{

    class GyroIOInputs implements LoggableInputs{
        public boolean connected = false;
        public Rotation2d yawPosition = new Rotation2d();
        public double yawPositionRadians = 0.0;
        public double yawPositionDegrees = 0.0;
        public Rotation2d pitchPosition = new Rotation2d();
        public Rotation2d rollPosition = new Rotation2d();

        @Override
        public void toLog(LogTable table){
            table.put("connected", connected);
            table.put("yawPosition", yawPosition);
            table.put("yawPositionRadians", yawPositionRadians);
            table.put("yawPositionDegrees", yawPositionDegrees);
            table.put("pitchPosition", pitchPosition);
            table.put("rollPosition", rollPosition);
        }

        @Override
        public void fromLog(LogTable table){
            connected = table.get("connected", connected);
            yawPosition = table.get("yawPosition", yawPosition);
            yawPositionRadians = table.get("yawPositionRadians", yawPositionRadians);
            yawPositionDegrees = table.get("yawPositionDegrees", yawPositionDegrees);
            pitchPosition = table.get("encoderVelocity", pitchPosition);
            rollPosition = table.get("encoderVelocity", rollPosition);
        }
    }

    void updateInputs(GyroIOInputs inputs);

}
