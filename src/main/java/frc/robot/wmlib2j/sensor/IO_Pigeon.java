// Written by WindingMotor as part of the wmlib2j library.

package frc.robot.wmlib2j.sensor;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.geometry.Rotation2d;

public class IO_Pigeon implements IO_GyroBase{

    // The AHRS class
    private final Pigeon2 pigeon;

    /**
     * Constructor for the IO_GyroReal class. Creates an instance of the AHRS class and calibrates it.
    */
    public IO_Pigeon(){
        // Create an instance of the AHRS class
        pigeon = new Pigeon2(13, "rio");

        // Set update frequency to 100Hz
        pigeon.getYaw().setUpdateFrequency(100);
        pigeon.getGravityVectorZ().setUpdateFrequency(100);
    }
    
    /**
     * Returns the rotation of the robot as an angle in radians, with a range from 0 to 2pi
     * @return Robots current rotation as an angle in radians.
    */
    private Rotation2d getYaw(){
        return pigeon.getRotation2d();
    }

    /**
     * Updates the inputs with the latest values.
     * @param inputs The inputs to update.
    */
    @Override
    public void updateInputs(GyroIOInputs inputs){
        inputs.connected = true;
        inputs.yawPosition = getYaw();
        inputs.yawPositionRadians = getYaw().getRadians();
        inputs.yawPositionDegrees = getYaw().getDegrees();
        inputs.pitchPosition = new Rotation2d();
        inputs.rollPosition = new Rotation2d();
    }

    /**
     * Calibrates the AHRS sensor by running the calibration process in a separate thread.
     * Adds a delay of 1 second, and resets and then zeros the yaw value of the sensor.
    */
    public void calibrateAndReset(){
        DriverStation.reportWarning("Pigeon calibration not implemented!", false);
    }

}
