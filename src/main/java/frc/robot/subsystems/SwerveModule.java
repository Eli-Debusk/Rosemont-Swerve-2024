package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;
import frc.rosemont.util.RoboMath;
import frc.rosemont.util.hardware.NEOBrushlessMotor;
import frc.rosemont.util.profiles.DefaultSwerveModuleProfile;

//(#) Swerve Module Class, (using NEO Brushless Motors, and CTRE-CANCoder Absolute Encoder)
public class SwerveModule {
    
    ////DEVICE INITIALIZATION

    private final NEOBrushlessMotor driveNEO, pivotNEO;

    private final RelativeEncoder driveEncoder, pivotEncoder;
    private final CANCoder absoluteEncoder;

    ////CLASS INITIALIZATIONS

    public SwerveModule (DefaultSwerveModuleProfile profile) {

        ////DEVICE DECLARATION

        driveNEO = new NEOBrushlessMotor(profile.driveCID);
        pivotNEO = new NEOBrushlessMotor(profile.pivotCID);

        driveEncoder = driveNEO.getEncoder(); //Uses the NEO's built-in encoder
        pivotEncoder = pivotNEO.getEncoder(); //Uses the NEO's built-in encoder
        absoluteEncoder = new CANCoder(profile.absEncoderCID);

        ////DEVICE CONFIGURATION

        driveNEO.setInverted(profile.driveReversed); //(i) Config direction of the drive motor
        pivotNEO.setInverted(profile.pivotReversed); //(i) Config direction of the pivot motor

        driveNEO.setIdleMode(IdleMode.kBrake); //(i) Config the brake|coast mode of the drive motor

        //(f) -> Configuring conversion factors for encoder readings
        driveEncoder.setPositionConversionFactor(SwerveConstants.DriveRotationToMeter);
        driveEncoder.setVelocityConversionFactor(SwerveConstants.DriveRPMToMPS);

        pivotEncoder.setPositionConversionFactor(SwerveConstants.PivotRotationToRadians);
        pivotEncoder.setVelocityConversionFactor(SwerveConstants.PivotRPMToRPS);

        //(i) Configuring PID Controller for the pivot motor
        pivotNEO.configPIDController(SwerveConstants.kPivotProportional, 0, 0);
        pivotNEO.configPIDControllerCI(-Math.PI, Math.PI);
    } 

    ////FEEDBACK FUNCTIONS

    //(f) -> Returns the current position of the drive encoder
    public double getDrivePosition() { 
        return driveEncoder.getPosition();
    }

    //(f) -> Returns the current velocity of the drive encoder
    public double getDriveVelocity() { 
        return driveEncoder.getVelocity();
    }

    //(f) -> Returns the current position of the pivot encoder
    public double getPivotPosition() { 
        return pivotEncoder.getPosition();  
    }

    //(f) -> Returns the current velocity of the pivot encoder
    public double getPivotVelocity() { 
        return pivotEncoder.getVelocity();  
    }

    //(f) -> Returns the current position of the absolute encoder
    public double getAbsolutePosition() { 
        return absoluteEncoder.getAbsolutePosition();
    }

    //(f) -> Returns SwerveModulePosition
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getPivotPosition()));
    }

    //(!) ONLY USE FOR TELEMETRY, NOT FOR CALCULATIONS
    public double[] reportEncoderData() { //(f) -> Returns an array of encoder data that can be used for telemetry
        return new double[] {
            driveEncoder.getPosition(),
            driveEncoder.getVelocity(), 
            pivotEncoder.getPosition(), 
            pivotEncoder.getVelocity(), 
            absoluteEncoder.getAbsolutePosition()
        };
    }

    public double[] reportMotorTempuratures() {
        return new double[] {
            driveNEO.getMotorTemperature(),
            pivotNEO.getMotorTemperature()
        };
    }

    ////MOVEMENT FUNCTIONS

    //(f) -> Retrieves the current SwerveModuleState given the current motor information
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getPivotPosition()));
    }

    //(f) -> Used to set the desired state of the module
    public void setModuleState(SwerveModuleState moduleState) { 

        //(f) -> Discards miniscule speed values
        if (Math.abs(moduleState.speedMetersPerSecond) < 0.001) {
            return;
        }

        //(f) -> Runs an optimization algorithm to reduce travel distance of the pivot motor
        moduleState = SwerveModuleState.optimize(moduleState, getModuleState().angle); 

        /* 
            (f) -> Gets the wanted velocity of the drive motor and divides it by the maximum speed 
            (f) -> of the chassis to get the desired power output of the motor.
        */
        driveNEO.set(RoboMath.clip(moduleState.speedMetersPerSecond / SwerveConstants.kMaxPhysicalSpeed, -1, 1));

        //(f) -> Sets the PID Output of the pivot motor to the desired state
        pivotNEO.runToPosition(moduleState.angle.getRadians());
    }

    //(f) -> Uses the relative encoder to zero the wheel angle 
    public void zeroModule() {
        driveNEO.stopMotor();
        pivotNEO.runToPosition(0);
    }

    ////UTIL FUNCTIONS

    //(f) -> Resets the relative encoder's position to 0
    public void resetEncoders() {
        driveEncoder.setPosition(0);
        pivotEncoder.setPosition(0);
    }

    //(f) -> Stops the drive and pivot motors
    public void stopMotors() {
        driveNEO.stopMotor();
        pivotNEO.stopMotor();
    }

    //(f) -> Sets the pivot's relEncoder position to the current absEncoder position
    public void zeroPivotEncoderToAbs() {
        pivotEncoder.setPosition(getAbsolutePosition());
    }
}