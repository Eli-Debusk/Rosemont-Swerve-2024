package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExampleShooter;

public class OperateShooterPower extends Command {

    private final ExampleShooter shooterSubsystem;
    private final double shooterPower;

    public OperateShooterPower(ExampleShooter shooterSubsystem, double shooterPower) {
        this.shooterSubsystem = shooterSubsystem;
        this.shooterPower = shooterPower;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        shooterSubsystem.setShooterPower(shooterPower);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopShooter();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}