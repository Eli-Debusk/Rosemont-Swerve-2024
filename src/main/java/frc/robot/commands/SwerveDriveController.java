package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TeleOPConstants;
import frc.robot.subsystems.SwerveDrive;
import frc.rosemont.util.RoboMath;
import frc.rosemont.util.controllers.DualFactorSpeedController;

//(#) The Swerve Drive TeleOP Command using documented configuration
public class SwerveDriveController extends Command {
  
  ////VARIABLE INITIALIZATION

  private final SwerveDrive swerveDriveSystem;

  private final Supplier<Double> speedXSupplier, speedYSupplier, speedRSupplier, speedMSupplier;
  private final Supplier<Boolean> resetPivotSupplier;

  private final SlewRateLimiter translationRateLimiter, angularRateLimiter;

  private final DualFactorSpeedController translationDualSpeedController, angularDualSpeedController;

  ////CLASS INITIALIZATION
  
  public SwerveDriveController(
    SwerveDrive subsystem,
    CommandXboxController controller
  ) {

    //(i) Defining Variables
    this.swerveDriveSystem = subsystem;

    this.speedXSupplier = () -> controller.getLeftX();
    this.speedYSupplier = () -> -controller.getLeftY();
    this.speedRSupplier = () -> controller.getRightX();
    this.speedMSupplier = () -> controller.getRightTriggerAxis();
    this.resetPivotSupplier = () -> controller.x().getAsBoolean();

    this.translationRateLimiter = new SlewRateLimiter(SwerveConstants.kMaxPhysicalAccelerationTeleOP);
    this.angularRateLimiter = new SlewRateLimiter(SwerveConstants.kMaxAngularAccelerationTeleOP);

    this.translationDualSpeedController = new DualFactorSpeedController(
      SwerveConstants.kNormalPhysicalSpeedTeleOP, 
      SwerveConstants.kFastPhysicalSpeedTeleOP
    );

    this.angularDualSpeedController = new DualFactorSpeedController(
      SwerveConstants.kNormalAngularSpeedTeleOP, 
      SwerveConstants.kFastAngularSpeedTeleOP
    );

    addRequirements(swerveDriveSystem);
  }

  ////EVENT FUNCTIONS

  //(f) -> Runs once when the command is started
  @Override
  public void initialize() {
    //swerveDriveSystem.zeroModulePivotPositions();
  }

  //(f) -> Runs periodically when the command is running
  @Override
  public void execute() {

    //(f) -> Retrives values from the speed suppliers
    double xSpeed = speedXSupplier.get();
    double ySpeed = speedYSupplier.get();
    double rSpeed = speedRSupplier.get();

    //(f) -> Uses a function to apply a "deadzone" to account for hardware errors
    xSpeed = RoboMath.applyDeadband(xSpeed, TeleOPConstants.kSpeedDeadband);
    ySpeed = RoboMath.applyDeadband(ySpeed, TeleOPConstants.kSpeedDeadband);
    rSpeed = RoboMath.applyDeadband(rSpeed, TeleOPConstants.kSpeedDeadband);

    //(f) -> Applies the Rate Limiters and the Dual Factor Speed Controllers to get a final velocity output
    xSpeed = translationRateLimiter.calculate(xSpeed) * translationDualSpeedController.calculate(speedMSupplier.get());
    ySpeed = translationRateLimiter.calculate(ySpeed) * translationDualSpeedController.calculate(speedMSupplier.get());
    rSpeed = angularRateLimiter.calculate(rSpeed) * angularDualSpeedController.calculate(speedMSupplier.get());

    //(i) Constructing new ChassisSpeeds object
    ChassisSpeeds chassisSpeeds;

    //(f) -> Definining the ChassisSpeeds object using velocity variables and the robot Rotation2D
    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      xSpeed, ySpeed, rSpeed, swerveDriveSystem.getRotation2D()
    );

    //(f) -> Converts the ChassisSpeeds into an array of SwerveModuleStates
    SwerveModuleState[] swerveStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(chassisSpeeds);

    //(i) Updating the desired swerve drive state
    swerveDriveSystem.setDriveState(swerveStates);

    //(f) -> Using the boolean supplier to conditionally reset the module pivot zero position
    if(resetPivotSupplier.get()) {
      swerveDriveSystem.zeroModulePivotPositions();
    } else {}
  }

  //(f) -> Runs once when the command is ended
  @Override
  public void end(boolean interrupted) {
    swerveDriveSystem.stopModules();
  }

  //(f) -> Ends the command conditionally
  @Override
  public boolean isFinished() {
    return false;
  }
}