// Written by WindingMotor as part of the wmlib2j library.

package frc.robot.util;
import edu.wpi.first.wpilibj.DigitalInput;

public class IRBeamBreak{

    private final DigitalInput sensor;

    /**
     * Creates a new IRBeamBreak digital IR sensor object.
     * @param channel The digital input channel
    */
    public IRBeamBreak(int channel){
        sensor = new DigitalInput(channel);
    }

    /**
     * Get the the current state of the sensor.
     * Returns true if the sensor is triggered (beam is broken).
     * @return  The state obtained from the sensor
    */
    public boolean getState() {
        return !sensor.get();
    }
}