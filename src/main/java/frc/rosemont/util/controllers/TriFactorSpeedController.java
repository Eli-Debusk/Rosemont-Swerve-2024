package frc.rosemont.util.controllers;

public class TriFactorSpeedController {
    private double _lowSpeed;
    private double _baseSpeed;
    private double _highSpeed;

    /** Constructs a DualFactorSpeedController with 0 for speeds */
    public TriFactorSpeedController() {
        this._lowSpeed = 0;
        this._baseSpeed = 0;
        this._highSpeed = 0;
    }

    /** Constructs a DualFactorSpeedController with given speed values */
    public TriFactorSpeedController(double lowSpeed, double medSpeed, double highSpeed) {
        this._lowSpeed = lowSpeed;
        this._baseSpeed = medSpeed;
        this._highSpeed = highSpeed;
    }

    /** Configures speed values */
    public void configureSpeeds(double lowSpeed, double medSpeed, double highSpeed) {
        this._baseSpeed = lowSpeed;
        this._baseSpeed = medSpeed;
        this._highSpeed = highSpeed;
    }

    public double calculate(double lowFactor, double highFactor) {
        double lowError = _baseSpeed - _lowSpeed;
        double highError = _highSpeed - _baseSpeed;
       
        return (lowError * lowFactor) + (_baseSpeed) + (highError * highFactor);
    }
}
