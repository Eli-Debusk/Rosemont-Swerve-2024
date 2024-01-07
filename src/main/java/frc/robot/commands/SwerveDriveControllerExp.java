package frc.robot.commands;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveDrive;
import frc.rosemont.util.controllers.SuppliedController;
import frc.rosemont.util.controllers.SwerveChassisController;

//(#) Swerve Drive TeleOP Command (EXPIREMENTAL - Not using documented methods; UNTESTED)
public class SwerveDriveControllerExp extends Command {
  
  private final SwerveDrive swerveDriveSystem;

  private final SwerveChassisController chassisController;

  private final SuppliedController controller;

  public SwerveDriveControllerExp(SwerveDrive subsystem, SuppliedController controller) {
    this.swerveDriveSystem = subsystem;

    this.chassisController = new SwerveChassisController(true);
    this.chassisController.defaultConfiguration();

    this.controller = controller;

    addRequirements(swerveDriveSystem);
  }

  @Override
  public void initialize() {
    //swerveDriveSystem.zeroModules();
  }

  @Override
  public void execute() {
    controller.update();
    chassisController.setSwerveSpeeds(controller, swerveDriveSystem.getRotation2D());

    SwerveModuleState[] swerveStates = SwerveConstants.swerveKinematics.toSwerveModuleStates(chassisController.getSwerveSpeeds());

    swerveDriveSystem.setDriveState(swerveStates);
  }

  @Override
  public void end(boolean interrupted) {
    swerveDriveSystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}