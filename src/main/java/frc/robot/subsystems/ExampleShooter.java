package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.rosemont.util.hardware.Falcon500;

//#EXAMPLE SHOOTER SUBSYSTEM
public class ExampleShooter extends SubsystemBase {

    ////DEVICE INITIALIZATION
    private Falcon500 leftSpinner, rightSpinner;
    
    ////CLASS INITIALIZATION
    public ExampleShooter() {
        
        ////DEVICE DECLARATION
        leftSpinner = new Falcon500(12); //Left Falcon500
        rightSpinner = new Falcon500(13); //Right Falcon500

        ////DEVICE CONFIGURATION
        leftSpinner.setInverted(false); //Left Falcon500 Direction
        rightSpinner.setInverted(true); //Right Falcon500 Direction
    }

    ////MOVEMENT FUNCTIONS
    //Setting motor output
    public void setShooterPower(double power) {
        leftSpinner.set(ControlMode.PercentOutput, power); //Left Falcon500 Power
        rightSpinner.set(ControlMode.PercentOutput, power); //Right Falcon500 Power
    }

    //Stopping motor output
    public void stopShooter() {
        leftSpinner.set(ControlMode.PercentOutput, 0); //Stopping Left Falcon500
        rightSpinner.set(ControlMode.PercentOutput, 0); //Stopping Right Falcon500
    }
}