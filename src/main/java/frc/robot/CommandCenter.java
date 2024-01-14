package frc.robot;

import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.TeleOPConstants;
import frc.robot.subsystems.SwerveDrive;

//(#) The Robot Command Center to schedule commands and operate the robot
public class CommandCenter {
  
  public final SwerveDrive swerveDrive = new SwerveDrive();

  public final CommandXboxController controller = new CommandXboxController(TeleOPConstants.kDriveControllerPort);

  public CommandCenter() {

    // swerveDrive.setDefaultCommand(new SwerveDriveController(
    //   swerveDrive, 
    //   controller
    // ));

    configureBindings();
  }

  private void configureBindings() {
    ////CONFIGURING BUTTON BINDINGS
    //(f) -> Binds x (whenPressed) to resetting swerve module positions
    controller.x(new EventLoop()).onTrue(new InstantCommand(() -> swerveDrive.resetModules()));
    
    //(f) -> Binds start (whenPressed) to resetting swerve drive heading
    controller.start(new EventLoop()).onTrue(new InstantCommand(() -> swerveDrive.resetHeading()));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
