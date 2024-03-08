// Copyright (c) 2024 : FRC 2106 : The Junkyard Dogs
// https://github.com/WindingMotor
// https://www.team2106.org

// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * The MathCalc class provides functionality to find the best RPM and angle for a projectile to hit
 * a target based on the horizontal distance to the target. It reads measurements from a CSV file
 * where each row contains RPM, angle, and the maximum horizontal distance the projectile can
 * travel.
 */
public class MathCalc {

	/**
	 * Calculate the angle required for hitting the target based a linear equation and the distance to
	 * the target https://www.desmos.com/calculator/btpumpltjv
	 *
	 * @param distanceToTarget The distance to the target
	 * @return The calculated required angle to hit the target.
	 * @units RPM & DEG
	 */
	public static double calculateInterpolate(double distanceToTarget) {
		return (50.5739 * Math.exp(-distanceToTarget)) + 36.3442;
	}

	/**
	 * Calculate the RPM required for hitting the target based on the current angle and distance to
	 * the target https://www.desmos.com/calculator/vu2iw2ssbj
	 * https://www.desmos.com/calculator/on4xzwtdwz
	 *
	 * @param currentAngle The current angle of the projectile motion
	 * @param distanceToTarget The distance to the target
	 * @return The calculated required RPM to hit the target.
	 * @author Dustin B
	 * @units RPM & DEG
	 */
	public static double physicsCalculate(double currentAngle, double distanceToTarget) {

		double g = -1 / 2 * 9.8 * Math.pow(distanceToTarget, 2);

		double den1 = Math.pow(Math.PI * 2 * 0.05 * 1 / 60, 2) * Math.pow(Math.cos(currentAngle), 2);

		double den2 =
				2.2 - (0.45 + 0.47 * Math.sin(currentAngle)) - distanceToTarget * Math.tan(currentAngle);

		double temp = g / den1;

		temp = temp / den2;

		return Math.sqrt(temp);
	}

	/**
	 * Generates a random integer between the min and max
	 *
	 * @param min The minimum value
	 * @param max The maximum value
	 * @return Random number
	 */
	public static int random(int min, int max) {
		return (int) (Math.random() * ((max - min) + 1)) + min;
	}

	/**
	 * Calculate the area of a triangle.
	 *
	 * @param x1, y1 coordinates of the first point
	 * @param x2, y2 coordinates of the second point
	 * @param x3, y3 coordinates of the third point
	 * @return The area of the triangle.
	 */
	public static double area(double x1, double y1, double x2, double y2, double x3, double y3) {
		return Math.abs((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0);
	}

	/**
	 * Check if a point is inside a triangle
	 *
	 * @param pointOne The first point
	 * @param pointTwo The second point
	 * @param pointThree The third point
	 * @param robotPoint The point to check
	 * @return true if the robot point is inside the triangle, false otherwise.
	 */
	public static boolean isPointInsideTriangle(
			Translation2d pointOne,
			Translation2d pointTwo,
			Translation2d pointThree,
			Translation2d robotPoint) {
		// Calculate area of triangle ABC
		double A =
				area(
						pointOne.getX(),
						pointOne.getY(),
						pointTwo.getX(),
						pointTwo.getY(),
						pointThree.getX(),
						pointThree.getY());

		// Calculate area of triangle PBC
		double A1 =
				area(
						robotPoint.getX(),
						robotPoint.getY(),
						pointTwo.getX(),
						pointTwo.getY(),
						pointThree.getX(),
						pointThree.getY());

		// Calculate area of triangle PAC
		double A2 =
				area(
						pointOne.getX(),
						pointOne.getY(),
						robotPoint.getX(),
						robotPoint.getY(),
						pointThree.getX(),
						pointThree.getY());

		// Calculate area of triangle PAB
		double A3 =
				area(
						pointOne.getX(),
						pointOne.getY(),
						robotPoint.getX(),
						robotPoint.getY(),
						pointTwo.getX(),
						pointTwo.getY());

		// Check if sum of A1, A2 and A3 is same as A
		return A == A1 + A2 + A3;
	}
}
