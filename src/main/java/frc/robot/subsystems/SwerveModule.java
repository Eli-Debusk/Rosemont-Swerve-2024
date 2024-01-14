package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.SwerveConstants;
import frc.rosemont.util.RoboMath;
import frc.rosemont.util.hardware.NEOBrushlessMotor;
import frc.rosemont.util.profiles.DefaultSwerveModuleProfile;

//(#) Swerve Module Class, (using NEO Brushless Motors, and CTRE-CANCoder Absolute Encoder)
public class SwerveModule {
    
    ////DEVICE CONSTRUCTION

    private final NEOBrushlessMotor driveNEO, pivotNEO;

    private final RelativeEncoder driveEncoder, pivotEncoder;
    private final CANcoder absoluteEncoder;

    ////CLASS INITIALIZATIONS

    public SwerveModule (DefaultSwerveModuleProfile profile) {

        ////DEVICE DECLARATION

        driveNEO = new NEOBrushlessMotor(profile.driveCID);
        pivotNEO = new NEOBrushlessMotor(profile.pivotCID);

        driveEncoder = driveNEO.getEncoder(); //(i) Uses the NEO's built-in encoder
        pivotEncoder = pivotNEO.getEncoder(); //(i) Uses the NEO's built-in encoder
        absoluteEncoder = new CANcoder(profile.absEncoderCID);

        ////DEVICE CONFIGURATION

        //(s) Motor Configuration
        driveNEO.setInverted(profile.driveReversed); //(f) Config direction of the drive motor
        pivotNEO.setInverted(profile.pivotReversed); //(f) Config direction of the pivot motor

        driveNEO.setIdleMode(IdleMode.kBrake); //(f) Config the brake|coast mode of the drive motor

        //(s) Encoder Configuration
        driveEncoder.setPositionConversionFactor(SwerveConstants.DriveRotationToMeter);
        driveEncoder.setVelocityConversionFactor(SwerveConstants.DriveRPMToMPS);

        pivotEncoder.setPositionConversionFactor(SwerveConstants.PivotRotationToRadians);
        pivotEncoder.setVelocityConversionFactor(SwerveConstants.PivotRPMToRPS);

        //(s) Motion Configuration
        pivotNEO.configPIDController(SwerveConstants.kPivotProportional, 0, 0);
        pivotNEO.configPIDControllerCI(-Math.PI, Math.PI);
    } 

    ////FEEDBACK FUNCTIONS

    //(s) Velocity Feedback
    public double getPivotVelocity() { 
        return pivotEncoder.getVelocity(); //(i) Returns value in meters per second
    }

    public double getDriveVelocity() { 
        return driveEncoder.getVelocity(); //(i) Returns value in radians per second
    }

    //(s) Position Feedback
    public double getDrivePosition() { 
        return driveEncoder.getPosition(); //(i) Returns continious meters
    }

    public double getPivotPosition() { 
        return pivotEncoder.getPosition(); //(i) Returns continious radians
    }

    public double getAbsolutePosition() { 
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble(); //(i) Returns radians between -π and π
    }

    //(s) SwerveModule Feedback
    public SwerveModulePosition getModulePosition() {
        //(i) Returns SwerveModulePosition with meters traveled and travel angle
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getPivotPosition()));
    }

    public SwerveModuleState getModuleState() {
        //(i) Returns SwerveModuleState with meters per second and travel angle
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getPivotPosition()));
    }

    ////MOVEMENT FUNCTIONS

    public void setDesiredModuleState(SwerveModuleState moduleState) { 

        //(s) Power calculations

        //(f) Discards miniscule power values
        if (Math.abs(moduleState.speedMetersPerSecond) < 0.001) {
            return;
        }

        //(f) Runs algorithm to minimize pivot distance
        moduleState = SwerveModuleState.optimize(moduleState, getModuleState().angle); 

        //(s) Motor Outputs

        //(i) Sets the drive motor's power to a value corrosponding to the motor's max speed and desired speed
        driveNEO.set(RoboMath.clip(moduleState.speedMetersPerSecond / SwerveConstants.kMaxPhysicalSpeed, -1, 1));
        
        //(i) Sets the pivot motor to run to a desired angle
        pivotNEO.runToPosition(moduleState.angle.getRadians(), getPivotPosition());
    }

    ////UTIL FUNCTIONS

    //(f) Currently unused
    public void resetEncoderPositions() {
        driveEncoder.setPosition(0); 
        pivotEncoder.setPosition(0);
    }

    //(f) Safety and Utility feature to stop motors
    public void stop() {
        driveNEO.stopMotor();
        pivotNEO.stopMotor();
    }

    //(f) Used to autozero when relative encoder resets
    public void autoZeroPivotEncoder() {
        pivotEncoder.setPosition(-getAbsolutePosition());
    }
}