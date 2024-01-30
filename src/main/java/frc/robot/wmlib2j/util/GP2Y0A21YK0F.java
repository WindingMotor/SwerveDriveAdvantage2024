
package frc.robot.wmlib2j.util;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants;

public class GP2Y0A21YK0F{

    private final AnalogInput analogInput;
    private long lastTime;

    /**
     * Creates a new GP2Y0A21YK0F analog sensor object.
     * @param channel The analog input channel
    */
    public GP2Y0A21YK0F(int channel){
        analogInput = new AnalogInput(channel);
        lastTime = System.currentTimeMillis();
    }

    /**
     * Returns the distance calculated in centimeters from the analog input voltage.
     * @return          The calculated distance in centimeters
    */
    public int getDistanceCentimeters(){

        if(System.currentTimeMillis() <= lastTime + 20){
            try{
                Thread.sleep(20);
            }catch(InterruptedException e){ e.printStackTrace(); }
        }

        lastTime = System.currentTimeMillis();

        double rawVoltage = analogInput.getVoltage();
        int distance = (int)(4800 / (rawVoltage - 20));

        if(distance > 80){
            return 81;
        }else if(distance < 10){
            return 9;
        }else{
            return distance;
        }
    }

    /**
     * Calculates the distance in inches based on the distance in centimeters.
     * @return The calculated distance in inches
    */
    public double getDistanceInches(){
        return getDistanceCentimeters() / 2.54;
    }

    /**
     * Calculate the distance in meters based on the distance in centimeters.
     * @return The distance in meters
    */
    public double getDistanceMeters(){
        return getDistanceCentimeters() / 100.0;
    }

    public boolean isWithinRange(){
        double measurement = getDistanceCentimeters();
        if(measurement > Constants.Beluga.SENSOR_TOLERANCE_MIN_CENTIMETERS && measurement < Constants.Beluga.SENSOR_TOLERANCE_MAX_CENTIMETERS){
            return true;
        }else{
            return false;

        }
    }

}
