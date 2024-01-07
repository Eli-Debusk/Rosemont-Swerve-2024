package frc.rosemont.util.hardware;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;

//#This little buddy is what we will be using for the NeoSwerveDrive in 2024
public class NEOBrushlessMotor extends CANSparkMax {

    private PIDController _controller = new PIDController(0, 0, 0);

    public NEOBrushlessMotor(int deviceId) {
        super(deviceId, MotorType.kBrushless);
    }

    public void configPIDController(double kP, double kI, double kD) {
        _controller.setPID(kP, kI, kD);
    }

    public void configPIDControllerCI(double minInput, double maxInput) {
        _controller.enableContinuousInput(minInput, maxInput);
    }

    public void runToPosition(double position) {
        this.set(_controller.calculate(position, getEncoder().getPosition()));
    }
}
