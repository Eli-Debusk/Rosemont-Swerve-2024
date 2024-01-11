package frc.rosemont.util.controllers;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants.SwerveConstants;
import frc.rosemont.util.RoboMath;

//#EXPIREMENTAL: Premade SwerveChassisController, similar to MecanumChassisController in git-rosemont-templates
public class SwerveChassisController {

    ////VARIABLES

    //All of these varaibles are temporary, and will be overwritten when setSwerveSpeeds() is called
    private double _xSpeed = 0;
    private double _ySpeed = 0;
    private double _rSpeed = 0;
    private double _lowSpeedModifier = 0;
    private double _highSpeedModifier = 0;
    private Rotation2d _heading;

    //Objects and Config Variables
    private double _deadband = 0.15; //Base deadband is 0.15

    private boolean triSpeedEnabled; //Stored boolean to check if tri-speed is enabled

    //Slew Rate Limiters for smoothed acceleration
    private SlewRateLimiter _xLimiter, _yLimiter;
    private SlewRateLimiter _angularAccelerator;

    //Used for tri-Speed Mode and to calculate the output speed
    private TriFactorSpeedController _triSpeedCardinalController;
    private TriFactorSpeedController _triSpeedAngularController;

    ////CLASS INITIALIZER
    public SwerveChassisController(boolean multiSpeedEnabled) {
        this.triSpeedEnabled = multiSpeedEnabled; //initializing tri-speed boolean

        //initializing SlewRateLimiters with default configuration
        this._xLimiter = new SlewRateLimiter(3); //Cardinal X acceleration
        this._yLimiter = new SlewRateLimiter(3); //Cardinal Y acceleration
        this._angularAccelerator = new SlewRateLimiter(3); //Angular acceleration

        //initializing tri-speed controllers with default configuration
        this._triSpeedCardinalController = new TriFactorSpeedController(5, 5, 5); //Cardinal tri-Speed Controller
        this._triSpeedAngularController = new TriFactorSpeedController(5, 5, 5); //Angular tri-Speed Controller
    }

    ////CONFIGURATION FUNCTIONS
    //Tri Speed Controller Configuration
    public void configTriSpeedControllers(double[] cardinalSpeeds, double[] angularSpeeds) {
        //Establishes configured values for cardinal and angular Tri-speed
        _triSpeedCardinalController.configureSpeeds(cardinalSpeeds[0], cardinalSpeeds[1],cardinalSpeeds[2]); //Cardinal tri-Speed Controller
        _triSpeedAngularController.configureSpeeds(angularSpeeds[0], angularSpeeds[1], angularSpeeds[3]); //Angular tri-Speed Controller
    }

    //Slew Rate Limiter Configuration
    public void configAccelerators(double cardinalAccelLimit, double angularAccelLimiter) {
        //Overwrites default SLRs for acceleration with new rate limits
        _xLimiter = new SlewRateLimiter(cardinalAccelLimit); //Cardinal X acceleration
        _yLimiter = new SlewRateLimiter(cardinalAccelLimit); //Cardinal Y acceleration
        _angularAccelerator = new SlewRateLimiter(angularAccelLimiter); //Angular acceleration
    }

    //Deadband Configuration
    public void configSafetyMeasures(double deadband) {
        _deadband = deadband; //Deadband configuration for controller inconsistencies
    }

    public void defaultConfiguration() {
        _triSpeedCardinalController.configureSpeeds(
            SwerveConstants.kSlowPhysicalSpeedTeleOP, 
            SwerveConstants.kNormalPhysicalSpeedTeleOP,
            SwerveConstants.kFastPhysicalSpeedTeleOP
            ); //Cardinal Tri-Speed Controller
        _triSpeedAngularController.configureSpeeds(
            SwerveConstants.kSlowAngularSpeedTeleOP, 
            SwerveConstants.kNormalAngularSpeedTeleOP, 
            SwerveConstants.kFastAngularSpeedTeleOP
            ); //Angular Tri-Speed Controller

        _deadband = 0.15; //Deadband configuration for controller inconsistencies
    }

    ////SET SWERVE SPEEDS
    //Sets swerve movement values using given input values
    public void setSwerveSpeeds(double xSpeed, double ySpeed, double rSpeed, double sHighModifier, double sLowModifier, Rotation2d heading) {
        this._xSpeed = xSpeed; //Chassis X-Speed
        this._ySpeed = ySpeed; //Chassis Y-Speed
        this._rSpeed = rSpeed; //Chassis Rotation-Speed
        this._lowSpeedModifier = sLowModifier; //Chassis Low Speed Modifier (for tri-speed)
        this._lowSpeedModifier = sHighModifier; //Chassis High Speed Modifier (for tri-speed)
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
        this._highSpeedModifier = controller.rt; //Chassis Speed Modifier (for tri-speed)
        this._lowSpeedModifier = controller.lt; //Chassis Speed Modifier (for tri-speed)
        this._heading = heading; //Chassis Rotation2d
    }

    ////GETS CALCULATED SWERVE SPEEDS
    //gets a ChassisSpeeds object from stored swerve movement values
    public ChassisSpeeds getSwerveSpeeds() {
        double x = _xSpeed; //Chassis X-Speed
        double y = _ySpeed; //Chassis Y-Speed
        double r = _rSpeed; //Chassis Rotation-Speed
        double sl = _lowSpeedModifier; //Chassis Speed Modifier (for tri-speed)
        double sh = _highSpeedModifier; //Chassis Speed Modifier (for tri-speed)

        //Modifying speed values to add deadband ignorance
        x = RoboMath.applyDeadband(x, _deadband); //X Deadband (lx)
        y = RoboMath.applyDeadband(y, _deadband); //Y Deadband (ly)
        r = RoboMath.applyDeadband(r, _deadband); //Rotation Deadband (rx)

        //Checks for tri-Speed condition then proceeds
        if(triSpeedEnabled) {
            x = _xLimiter.calculate(x) * _triSpeedCardinalController.calculate(sl, sh); //tri X-Speed
            y = _yLimiter.calculate(y) * _triSpeedCardinalController.calculate(sl, sh); //tri Y-Speed
            r = _angularAccelerator.calculate(r) * _triSpeedAngularController.calculate(sl, sh); //tri Rotation-Speed
        } else {
            x = _xLimiter.calculate(x) * 5; //One factor X-Speed
            y = _yLimiter.calculate(y) * 5; //One factor Y-Speed
            r = _angularAccelerator.calculate(r) * 5; //One factor Rotation-Speed
        }

        //Returns chassis-speeds using the calculated speeds
        return ChassisSpeeds.fromFieldRelativeSpeeds(y, x, r, _heading); 
    }
}
