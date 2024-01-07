package frc.rosemont.util.controllers;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class SuppliedController {
    private final Supplier<Double> _leftYSupplier, _leftXSupplier;

    private final Supplier<Double> _rightYSupplier, _rightXSupplier;

    private final Supplier<Double> _leftTriggerSupplier, _rightTriggerSupplier;

    public double ly, lx, ry, rx, lt, rt;

    public SuppliedController(XboxController controller, boolean reverseLeftY) {
        this._leftYSupplier = () -> reverseLeftY == true ? -controller.getLeftY() : controller.getLeftY();
        this._leftXSupplier = () -> controller.getLeftX();

        this._rightYSupplier = () -> controller.getRightY();
        this._rightXSupplier = () -> controller.getRightX();

        this._leftTriggerSupplier = () -> controller.getLeftTriggerAxis();
        this._rightTriggerSupplier = () -> controller.getRightTriggerAxis();
    }

    public SuppliedController(CommandXboxController controller, boolean reverseLeftY) {
        this._leftYSupplier = () -> reverseLeftY == true ? -controller.getLeftY() : controller.getLeftY();
        this._leftXSupplier = () -> controller.getLeftX();

        this._rightYSupplier = () -> controller.getRightY();
        this._rightXSupplier = () -> controller.getRightX();

        this._leftTriggerSupplier = () -> controller.getLeftTriggerAxis();
        this._rightTriggerSupplier = () -> controller.getRightTriggerAxis();
    }

    public void update() {
        this.ly = _leftYSupplier.get();
        this.lx = _leftXSupplier.get();

        this.ry = _rightYSupplier.get();
        this.rx = _rightXSupplier.get();

        this.lt = _leftTriggerSupplier.get();
        this.rt = _rightTriggerSupplier.get();
    }
}
