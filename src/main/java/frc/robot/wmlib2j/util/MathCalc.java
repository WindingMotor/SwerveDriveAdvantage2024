package frc.robot.wmlib2j.util;

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
    
    public static String CSV_FILE_PATH = "src/main/java/frc/robot/wmlib2j/csv/";
    /**
     * The Measurement class represents a single measurement containing
     * the RPM, angle, and distance values.
    */
    static class Measurement{
        public double rpm;
        public double angle;
        public double distance;

        /**
         * Constructs a new Measurement instance.
         * @param rpm      The revolutions per minute (RPM) value
         * @param angle    The angle in degrees
         * @param distance The maximum horizontal distance in meters
        */
        public Measurement(double rpm, double angle, double distance){
            this.rpm = rpm;
            this.angle = angle;
            this.distance = distance;
        }
    }

    /**
     * Finds the best match for RPM and angle based on the given target distance.
     * It reads the measurements from a CSV file and selects the measurement
     * with the distance closest to the target distance.
     *
     * @param targetDistance The target horizontal distance in meters
     * @return The best matching Measurement object containing RPM and angle
     * @throws IOException  If an I/O error occurs while reading the CSV file
     * @throws CsvException If an error occurs while parsing the CSV file
    */
    public static Measurement calculateBestMatch(double targetDistance) throws IOException, CsvException{

        // Path to the csv file containing the measurements in the order RPM, angle, distance
        String csvFile = CSV_FILE_PATH;

        // List to store the measurements from the csv file
        List<Measurement> measurements = new ArrayList<>();

        // Try-with-resources statement to ensure the CSVReader is closed after use
        try(CSVReader reader = new CSVReader(new FileReader(csvFile))){
            String[] nextLine;

            // Loop through each row in the CSV file
            while((nextLine = reader.readNext()) != null){
                // Parse the RPM, angle, and distance values from the row
                double rpm = Double.parseDouble(nextLine[0]);
                double angle = Double.parseDouble(nextLine[1]);
                double distance = Double.parseDouble(nextLine[2]);
                // Add a new Measurement object to the list with the parsed values
                measurements.add(new Measurement(rpm, angle, distance));
            }
        }

        // Initialize variables to track the best match and the smallest difference found so far
        Measurement bestMatch = null;
        double smallestDifference = Double.MAX_VALUE;

        // Iterate through each measurement to find the one closest to the target distance
        for(Measurement measurement : measurements){
            // Calculate the difference between the measurement distance and the target distance
            double difference = Math.abs(measurement.distance - targetDistance);
            // If the difference is less than the smallest difference found so far, update the best match
            if(difference < smallestDifference){
                bestMatch = measurement;
                smallestDifference = difference;
            }
        }
        return bestMatch;
    }

    /**
     * Represents a prediction containing the RPM and angle values.
    */
    static class Prediction{
        public double rpm;
        public double angle;

        /**
         * Constructs a new Prediction instance.
         * @param rpm   The revolutions per minute (RPM) value
         * @param angle The angle in degrees
        */
        public Prediction(double rpm, double angle){
            this.rpm = rpm;
            this.angle = angle;
        }
    }

    /**
     * Predicts the RPM and angle required for a projectile to hit a target
     * based on the horizontal distance to the target. This method uses data
     * from a CSV file containing RPM and angle measurements to perform
     * polynomial regression and predict the necessary shooting parameters.
     *
     * @param targetDistance The target horizontal distance in meters
     * @return A Prediction object containing the predicted RPM and angle
     * @throws IOException  If an I/O error occurs while reading the CSV file
     * @throws CsvException If an error occurs while parsing the CSV file
     */
    public static Prediction calculatePolynomialPrediction(double targetDistance) throws IOException, CsvException{

        String csvFile = "path/to/your/deploy/directory/measurements.CSV";
        WeightedObservedPoints obsRPM = new WeightedObservedPoints();
        WeightedObservedPoints obsAngle = new WeightedObservedPoints();

        try(CSVReader reader = new CSVReader(new FileReader(csvFile))){
            String[] nextLine;
            while((nextLine = reader.readNext()) != null){
                double distance = Double.parseDouble(nextLine[2]); // Assuming distance is the third column.
                double rpm = Double.parseDouble(nextLine[0]); // RPM is the first column.
                double angle = Double.parseDouble(nextLine[1]); // Angle is the second column.
                obsRPM.add(distance, rpm);
                obsAngle.add(distance, angle);
            }
        }

        // Create a polynomial curve fitter with a degree of 2, indicating quadratic regression.
        PolynomialCurveFitter fitter = PolynomialCurveFitter.create(2);

        // Fit the observed RPM points to a quadratic curve and retrieve the coefficients.
        double[] coeffRPM = fitter.fit(obsRPM.toList());

        // Fit the observed angle points to a quadratic curve and retrieve the coefficients.
        double[] coeffAngle = fitter.fit(obsAngle.toList());

        /*
         * Use the regression model coefficients to predict the RPM and angle for the given target distance.
         * The prediction is based on the quadratic equation derived from the regression analysis.
        */
        double predictedRPM = coeffRPM[0] + coeffRPM[1] * targetDistance + coeffRPM[2] * Math.pow(targetDistance, 2);
        double predictedAngle = coeffAngle[0] + coeffAngle[1] * targetDistance + coeffAngle[2] * Math.pow(targetDistance, 2);

        return new Prediction(predictedRPM, predictedAngle);
    }

    /**
     * Calculates the required launch angle and RPM to hit a static target.
     * @return A string describing the required angle in degrees and RPM.
    */
    public static Prediction calculateProjectileMotion(double targetDistance, double targetHeight){

        final double g = 9.81; // gravity in m/s^2
        final double v0 = 60 * Math.PI * 2 / 60; // initial velocity in m/s
        final double h = targetHeight; // height of target in m
        final double d = targetDistance; // distance to target in m
    
        // Compute time of flight using quadratic formula
        double discriminant = Math.pow(v0, 2) - 4 * g * h;
        if (discriminant < 0) {
            throw new IllegalArgumentException("Target is too high");
        }
        double t = (-v0 + Math.sqrt(discriminant)) / (2 * g);
    
        // Compute required wheel RPM and launch angle
        double rpm = 60 * Math.sqrt(2 * h / g);
        double angle = Math.toDegrees(Math.atan(Math.sqrt(2 * h / g)));
    
        return new Prediction(rpm, angle);
    }
}
