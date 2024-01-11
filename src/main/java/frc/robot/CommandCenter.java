package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.TeleOPConstants;
import frc.robot.commands.SwerveDriveController;
import frc.robot.subsystems.SwerveDrive;

//(#) The Robot Command Center to schedule commands and operate the robot
public class CommandCenter {
  
  public final SwerveDrive swerveDrive = new SwerveDrive();

  public final CommandXboxController controller = new CommandXboxController(TeleOPConstants.kDriveControllerPort);

  public CommandCenter() {

    swerveDrive.setDefaultCommand(new SwerveDriveController(
      swerveDrive, 
      controller
    ));

    // swerveDrive.setDefaultCommand(new SwerveDriveControllerExp(
    //   swerveDrive,
    //   new SuppliedController(controller, true)
    // ));

    configureBindings();
  }

  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return null;
  }
}
