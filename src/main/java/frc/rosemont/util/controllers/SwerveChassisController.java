package frc.rosemont.util.controllers;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.rosemont.util.RoboMath;

//#EXPIREMENTAL: Premade SwerveChassisController, similar to MecanumChassisController in git-rosemont-templates
public class SwerveChassisController {

    ////VARIABLES

    //All of these varaibles are temporary, and will be overwritten when setSwerveSpeeds() is called
    private double _xSpeed = 0;
    private double _ySpeed = 0;
    private double _rSpeed = 0;
    private double _speedModifier = 0;
    private Rotation2d _heading;

    //Objects and Config Variables
    private double _deadband = 0.15; //Base deadband is 0.15

    private boolean _dualSpeedEnabled; //Stored boolean to check if dual-speed is enabled

    //Slew Rate Limiters for smoothed acceleration
    private SlewRateLimiter _cardinalAccelerator; 
    private SlewRateLimiter _angularAccelerator;

    //Used for Dual-Speed Mode and to calculate the output speed
    private DualFactorSpeedController _dualSpeedCardinalController;
    private DualFactorSpeedController _dualSpeedAngularController;

    ////CLASS INITIALIZER
    public SwerveChassisController(boolean dualSpeedEnabled) {
        this._dualSpeedEnabled = dualSpeedEnabled; //initializing dual-speed boolean

        //initializing SlewRateLimiters with default configuration
        this._cardinalAccelerator = new SlewRateLimiter(3); //Cardinal acceleration
        this._angularAccelerator = new SlewRateLimiter(3); //Angular acceleration

        //initializing dual-speed controllers with default configuration
        this._dualSpeedCardinalController = new DualFactorSpeedController(5, 5); //Cardinal Dual-Speed Controller
        this._dualSpeedAngularController = new DualFactorSpeedController(5, 5); //Angular Dual-Speed Controller
    }

    ////CONFIGURATION FUNCTIONS
    //Dual Speed Controller Configuration
    public void configDualSpeedControllers(double[] cardinalSpeeds, double[] angularSpeeds) {
        //Establishes configured values for cardinal and angular dual-speed
        _dualSpeedCardinalController.configureSpeeds(cardinalSpeeds[0], cardinalSpeeds[1]); //Cardinal Dual-Speed Controller
        _dualSpeedAngularController.configureSpeeds(angularSpeeds[0], angularSpeeds[1]); //Angular Dual-Speed Controller
    }

    //Slew Rate Limiter Configuration
    public void configAccelerators(double cardinalAccelLimit, double angularAccelLimiter) {
        //Overwrites default SLRs for acceleration with new rate limits
        _cardinalAccelerator = new SlewRateLimiter(cardinalAccelLimit); //Cardinal acceleration
        _angularAccelerator = new SlewRateLimiter(angularAccelLimiter); //Angular acceleration
    }

    //Deadband Configuration
    public void configSafetyMeasures(double deadband) {
        _deadband = deadband; //Deadband configuration for controller inconsistencies
    }

    public void defaultConfiguration() {
        _dualSpeedCardinalController.configureSpeeds(6, 10); //Cardinal Dual-Speed Controller
        _dualSpeedAngularController.configureSpeeds(2 * Math.PI, 2.5 * Math.PI); //Angular Dual-Speed Controller

        _deadband = 0.15; //Deadband configuration for controller inconsistencies
    }

    ////SET SWERVE SPEEDS
    //Sets swerve movement values using given input values
    public void setSwerveSpeeds(double xSpeed, double ySpeed, double rSpeed, double sModifier, Rotation2d heading) {
        this._xSpeed = xSpeed; //Chassis X-Speed
        this._ySpeed = ySpeed; //Chassis Y-Speed
        this._rSpeed = rSpeed; //Chassis Rotation-Speed
        this._speedModifier = sModifier; //Chassis Speed Modifier (for dual-speed)
        this._heading = heading; //Chassis Rotation2d
    }

    /** sets movement values for getSwerveSpeeds() 
    *@param controller SuppliedController that will be used for angular and translation speeds (make sure to call .update())
    *@param heading Rotation2d of robot orientation
    */
    public void setSwerveSpeeds(SuppliedController controller, Rotation2d heading) {
        this._xSpeed = controller.lx; //Chassis X-Speed
        this._ySpeed = controller.ly; //Chassis Y-Speed
        this._rSpeed = controller.rx; //Chassis Rotation-Speed
        this._speedModifier = controller.rt; //Chassis Speed Modifier (for dual-speed)
        this._heading = heading; //Chassis Rotation2d
    }

    ////GETS CALCULATED SWERVE SPEEDS
    //gets a ChassisSpeeds object from stored swerve movement values
    public ChassisSpeeds getSwerveSpeeds() {
        double x = _xSpeed; //Chassis X-Speed
        double y = _ySpeed; //Chassis Y-Speed
        double r = _rSpeed; //Chassis Rotation-Speed
        double s = _speedModifier; //Chassis Speed Modifier (for dual-speed)

        //Modifying speed values to add deadband ignorance
        x = RoboMath.applyDeadband(x, _deadband); //X Deadband (lx)
        y = RoboMath.applyDeadband(y, _deadband); //Y Deadband (ly)
        r = RoboMath.applyDeadband(r, _deadband); //Rotation Deadband (rx)

        //Checks for Dual-Speed condition then proceeds
        if(_dualSpeedEnabled) {
            x = _cardinalAccelerator.calculate(x) * _dualSpeedCardinalController.calculate(s); //Dual X-Speed
            y = _cardinalAccelerator.calculate(y) * _dualSpeedCardinalController.calculate(s); //Dual Y-Speed
            r = _angularAccelerator.calculate(r) * _dualSpeedAngularController.calculate(s); //Dual Rotation-Speed
        } else {
            x = _cardinalAccelerator.calculate(x) * 5; //One factor X-Speed
            y = _cardinalAccelerator.calculate(y) * 5; //One factor Y-Speed
            r = _angularAccelerator.calculate(r) * 5; //One factor Rotation-Speed
        }

        //Returns chassis-speeds using the calculated speeds
        return ChassisSpeeds.fromFieldRelativeSpeeds(x, y, r, _heading); 
    }
}
