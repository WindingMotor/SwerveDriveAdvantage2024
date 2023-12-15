package frc.robot.wmlib2.sensor;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

public class IO_GyroReal implements IO_Gyro{

    private final AHRS ahrs;

    public IO_GyroReal(){
        ahrs = new AHRS(SPI.Port.kMXP);
        ahrs.calibrate();
        ahrs.reset();
    }

    // Returns an angle in radians from -pi to pi of the robot's rotation
    private Rotation2d getRotation2dWrappedRadians(){
        return new Rotation2d(MathUtil.angleModulus(-ahrs.getYaw()));
    }

    @Override
    public void updateInputs(GyroIOInputs inputs){
        inputs.connected = ahrs.isConnected();
        inputs.yawPosition = getRotation2dWrappedRadians();
        inputs.yawPositionRadians = getRotation2dWrappedRadians().getRadians();
        inputs.yawPositionDegrees = getRotation2dWrappedRadians().getDegrees();
        inputs.pitchPosition = new Rotation2d();
        inputs.rollPosition = new Rotation2d();
    }

}
