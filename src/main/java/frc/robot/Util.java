package frc.robot;

import edu.wpi.first.math.Pair;

/** Some utility functions. */
public class Util {
    /**
     * Applies a dead zone over a linear space.
     *
     * @param deadZone The zone centered around zero with range of [-dead zone, dead zone]
     * @param input The input to apply a dead zone too
     *
     * @return returns a value of either 0 or the input
     */
    public static Double applyLinearDeadZone(double deadZone, double input) {
        return Math.abs(input) > deadZone ? input : 0.0;
    }

    /**
     * Applies a deadZone over a circular space.
     *
     * @param deadZone The zone centered around zero with a radius of dead zone
     * @param x the input on the x axis
     * @param y the input on the y axis
     *
     * @return an array of size two of form [x, y] where either both x and y are zero or they both reflect the input
     */
    public static Pair<Double, Double> applyCircularDeadZone(double deadZone, double x, double y) {
        Double magnitude = Math.sqrt((x * x) + (y * y));
        Pair<Double, Double> ret  = new Pair<Double, Double>(x, y);
        Pair<Double, Double> zero = new Pair<Double, Double>(0.0, 0.0);

        return magnitude > deadZone ? ret : zero;
    }
} 
