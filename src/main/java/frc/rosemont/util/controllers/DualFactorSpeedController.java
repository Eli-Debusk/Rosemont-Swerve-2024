package frc.rosemont.util.controllers;

public class DualFactorSpeedController {
    private double _baseSpeed;
    private double _highSpeed;

    /** Constructs a DualFactorSpeedController with 0 for speeds */
    public DualFactorSpeedController() {
        this._baseSpeed = 0;
        this._highSpeed = 0;
    }

    /** Constructs a DualFactorSpeedController with given speed values */
    public DualFactorSpeedController(double lowSpeed, double highSpeed) {
        this._baseSpeed = lowSpeed;
        this._highSpeed = highSpeed;
    }

    /** Configures speed values */
    public void configureSpeeds(double lowSpeed, double highSpeed) {
        this._baseSpeed = lowSpeed;
        this._highSpeed = highSpeed;
    }

    /** Bridges the gap between low and high speed by a percentage of the difference
     * @param factor The amount factor (between 0 and 1) that is multiplied by the speeds difference
     * @return The low speed + a percentage of the difference decided by the factor
     */
    public double calculate(double factor) {
        double error = _highSpeed - _baseSpeed;

        return _baseSpeed + (error * factor);
    }
}
