package frc.rosemont.util;

public class RoboMath {

    /** Limits a value to a specified range 
     * @param value the value that will be affected
     * @param min the minimum limit
     * @param max the maximum limit
     * @return a value that is greater than a minimum and less than a maximum
     */
    public static double clip(double value, double min, double max) {
        return Math.max(min, Math.min(value, max));
    }

    /** Discards a value when that value is below a threshold
     * @param value the input value to manipulate
     * @param deadband the threshold value, where any value below this is discarded
     * @return the value after application of the threshold discarding
     */
    public static double applyDeadband(double value, double deadband) {
        return Math.abs(value) > deadband ? value : 0.0;
    }

    public static double headingRemainder(double angle) {
        return Math.IEEEremainder(angle, 360);
    }
}
