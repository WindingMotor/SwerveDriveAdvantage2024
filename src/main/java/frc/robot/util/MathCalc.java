// Written by WindingMotor as part of the wmlib2j library.

package frc.robot.util;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.apache.commons.math3.fitting.PolynomialCurveFitter;
import org.apache.commons.math3.fitting.WeightedObservedPoints;
import com.opencsv.CSVReader;
import com.opencsv.exceptions.CsvException;

/**
 * The MathCalc class provides functionality to find the best RPM and angle
 * for a projectile to hit a target based on the horizontal distance to the target.
 * It reads measurements from a CSV file where each row contains RPM, angle, and
 * the maximum horizontal distance the projectile can travel.
*/
public class MathCalc{


    /**
     * Calculate the angle required for hitting the target based a linear equation and the distance to the target
     * https://www.desmos.com/calculator/btpumpltjv
     * @param  distanceToTarget The distance to the target
     * @return                  The calculated required angle to hit the target.
     * @units                   RPM & DEG
    */
    public static double calculateInterpolate(double distanceToTarget){
        return( -6.55 * (distanceToTarget) ) + 59.75;
    }

    /**
     * Calculate the RPM required for hitting the target based on the current angle and distance to the target
     * https://www.desmos.com/calculator/vu2iw2ssbj
     * https://www.desmos.com/calculator/on4xzwtdwz
     * @param  currentAngle     The current angle of the projectile motion
     * @param  distanceToTarget The distance to the target
     * @return                  The calculated required RPM to hit the target.
     * @author                  Dustin B
     * @units                   RPM & DEG
    */
    public static double physicsCalculate(double currentAngle, double distanceToTarget){

        double g = -1/2 * 9.8 * Math.pow(distanceToTarget, 2);

	    double den1 = Math.pow(Math.PI * 2 * 0.05 * 1/60, 2) * Math.pow(Math.cos(currentAngle), 2);

	    double den2 = 2.2 - (0.45 + 0.47 * Math.sin(currentAngle)) - distanceToTarget * Math.tan(currentAngle);

	    double temp = g / den1; 

	    temp = temp / den2; 

	    return Math.sqrt(temp); 
    }

    public static int random(int min, int max) {
        return (int)(Math.random() * ((max - min) + 1)) + min;
    }
}
