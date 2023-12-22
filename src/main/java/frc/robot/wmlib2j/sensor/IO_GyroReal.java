package frc.robot.wmlib2j.sensor;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

public class IO_GyroReal implements IO_GyroBase{

    // The AHRS class
    private final AHRS ahrs;

    /**
     * Constructor for the IO_GyroReal class. Creates an instance of the AHRS class and calibrates it.
    */
    public IO_GyroReal(){
        // Create an instance of the AHRS class
        ahrs = new AHRS(SPI.Port.kMXP);

        // Calibrate and reset the gyro
        calibrateAndReset();
    }
    
    /**
     * Returns the rotation of the robot as an angle in radians, with a range from -pi to pi.
     * @return Robots current rotation as an angle in radians.
    */
    private Rotation2d getRotation2dWrappedRadians(){
        return new Rotation2d(MathUtil.angleModulus(-ahrs.getYaw()));
    }

    /**
     * Updates the inputs with the latest values.
     * @param inputs The inputs to update.
     */
    @Override
    public void updateInputs(GyroIOInputs inputs){
        inputs.connected = ahrs.isConnected();
        inputs.yawPosition = getRotation2dWrappedRadians();
        inputs.yawPositionRadians = getRotation2dWrappedRadians().getRadians();
        inputs.yawPositionDegrees = getRotation2dWrappedRadians().getDegrees();
        inputs.pitchPosition = new Rotation2d();
        inputs.rollPosition = new Rotation2d();
    }

    /**
     * Calibrates the AHRS sensor by running the calibration process in a separate thread.
     * Adds a delay of 1 second, and resets and then zeros the yaw value of the sensor.
    */
    public void calibrateAndReset(){
        DriverStation.reportWarning("Calibrating Gyro", false);
        new Thread(() -> {
            ahrs.calibrate();
            try{
                Thread.sleep(1000); // adding delay
            }catch(InterruptedException ex){
                DriverStation.reportError("Gyro Calibration Interrupted: " + ex.getMessage(), false);
            }
            ahrs.reset();
            ahrs.zeroYaw();
            DriverStation.reportWarning("Gyro Successfully Calibrated", false);
        }).start();
    }

}
