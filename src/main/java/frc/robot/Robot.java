// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.software_subsystems.Dashboard;

public class Robot extends TimedRobot {
  public static CommandCenter robotCommandContainer;

  public static Dashboard dashboard;

  @Override
  public void robotInit() {
    robotCommandContainer = new CommandCenter();
    dashboard = new Dashboard(robotCommandContainer.swerveDrive);
    dashboard.init();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    dashboard.periodic();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}
}
