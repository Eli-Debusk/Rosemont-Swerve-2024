package frc.robot.commands.SwerveDriveCommands;

import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TeleOPConstants;
import frc.robot.subsystems.SwerveDrive;
import frc.rosemont.util.RoboMath;
import frc.rosemont.util.controllers.DualFactorSpeedController;

public class SwerveDriveController extends Command {
  
  ////VARIABLE CONSTRUCTION

  private final SwerveDrive swerveDriveSystem;

  private final Supplier<Double> speedXSupplier, speedYSupplier, speedRSupplier, speedMSupplier;

  private final SlewRateLimiter xRateLimiter, yRateLimiter, angularRateLimiter;

  private final DualFactorSpeedController translationDualSpeedController, angularDualSpeedController;

  ////CLASS INITIALIZATION
  public SwerveDriveController(
    SwerveDrive subsystem,
    CommandXboxController controller
  ) {

    ////VARIABLE DECLARATION

    this.swerveDriveSystem = subsystem;

    this.speedXSupplier = ()  -> controller.getLeftX();
    this.speedYSupplier = ()  -> -controller.getLeftY();
    this.speedRSupplier = ()  -> controller.getRightX();
    this.speedMSupplier = ()  -> controller.getRightTriggerAxis();

    this.xRateLimiter = new SlewRateLimiter(SwerveConstants.kMaxPhysicalAccelerationTeleOP);
    this.yRateLimiter = new SlewRateLimiter(SwerveConstants.kMaxPhysicalAccelerationTeleOP);
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

  ////COMMAND STRUCTURE

  @Override
  public void initialize() {
    swerveDriveSystem.autoZeroModulePositions(); //(f) Ensures relative encoders retain zero position between resets
  }

  @Override
  public void execute() {

    double xSpeed = speedXSupplier.get();
    double ySpeed = speedYSupplier.get();
    double rSpeed = speedRSupplier.get();

    xSpeed = RoboMath.applyDeadband(xSpeed, TeleOPConstants.kSpeedDeadband);
    ySpeed = RoboMath.applyDeadband(ySpeed, TeleOPConstants.kSpeedDeadband);
    rSpeed = RoboMath.applyDeadband(rSpeed, TeleOPConstants.kSpeedDeadband);

    xSpeed = xRateLimiter.calculate(xSpeed) * translationDualSpeedController.calculate(speedMSupplier.get());
    ySpeed = yRateLimiter.calculate(ySpeed) * translationDualSpeedController.calculate(speedMSupplier.get());
    rSpeed = angularRateLimiter.calculate(rSpeed) * angularDualSpeedController.calculate(speedMSupplier.get());

    ChassisSpeeds chassisSpeeds;

    chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
      ySpeed, xSpeed, rSpeed, swerveDriveSystem.getRotation2D()
    );

    swerveDriveSystem.setDesiredChassisSpeeds(chassisSpeeds); //
  }

  @Override
  public void end(boolean interrupted) {
    swerveDriveSystem.stopModules(); //(f) Stops modules if command is interrupted
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}