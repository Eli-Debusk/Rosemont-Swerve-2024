package frc.rosemont.util.hardware;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;

//#This is the big guy
public class Falcon500 extends TalonFX {

    private PIDController _controller = new PIDController(0, 0, 0);

    public Falcon500(int deviceId) {
        super(deviceId);
    }

    public void configPIDController(double kP, double kI, double kD) {
        _controller.setPID(kP, kI, kD);
    }

    public void runToPosition(double position) {
        this.set(_controller.calculate(getPosition().getValueAsDouble(), position));
    }
}
