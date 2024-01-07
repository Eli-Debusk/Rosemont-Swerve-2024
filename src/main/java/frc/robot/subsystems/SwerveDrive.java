package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.rosemont.util.RoboMath;
import frc.rosemont.util.RosemontConstants.SwerveModulePositions;
import frc.rosemont.util.profiles.DefaultSwerveModuleProfile;

//(#) Full Swerve Drive Class (using four SDS MK4I Swerve Modules)
public class SwerveDrive extends SubsystemBase {

    ////DEVICE INITIALIZATION

    private final SwerveModule leftBack = new SwerveModule(new DefaultSwerveModuleProfile(SwerveModulePositions.LEFTBACK));
    private final SwerveModule leftFront = new SwerveModule(new DefaultSwerveModuleProfile(SwerveModulePositions.LEFTFRONT));
    private final SwerveModule rightBack = new SwerveModule(new DefaultSwerveModuleProfile(SwerveModulePositions.RIGHTBACK));
    private final SwerveModule rightFront = new SwerveModule(new DefaultSwerveModuleProfile(SwerveModulePositions.RIGHTFRONT));

    //(i) KuaiLabs NavX Gyroscope
    private final AHRS gyroscope = new AHRS(Port.kMXP);
    
    ////CLASS INITIALIZATION
    
    public SwerveDrive() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading(); //(f) Resets gyroscope angle
            } catch (Exception e) {}
        })
        .start();
    }

    ////UTIL FUNCTIONS

    public void zeroHeading() {
        gyroscope.reset(); 
    }

    ////FEEDBACK FUNCTIONS

    //(f) -> Returns IEEERemainder value from gyroscope angle and value
    public double getHeading() {
        return RoboMath.headingRemainder(gyroscope.getAngle());
    }

    //(f) -> Retrieves robot Rotation2D value
    public Rotation2d getRotation2D() {
        return Rotation2d.fromDegrees(getHeading());
    }

    //(f) -> Returns an Array of absolute encoder values
    public double[] getAbsoluteModulePositions() {
        return new double[] {
            leftFront.getAbsolutePosition(),
            leftBack.getAbsolutePosition(),
            rightFront.getAbsolutePosition(),
            rightBack.getAbsolutePosition()
        };
    }

    //(f) -> Returns an Array of relative encoder values
    public double[] getRelativeModulePositions() {
        return new double[] {
            leftFront.getPivotPosition(),
            leftBack.getPivotPosition(),
            rightFront.getPivotPosition(),
            rightBack.getPivotPosition()
        };
    }

    //(f) -> Returns Module SwerveModulePositions as an array
    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
                leftFront.getModulePosition(),
                rightFront.getModulePosition(),
                leftBack.getModulePosition(),
                rightBack.getModulePosition()                
        };
    }

    //(f) -> Returns Module motor tempuratures as an array
    public double[] reportMotorTemps() {
        return new double[] {
            leftBack.reportMotorTempuratures()[0],
            leftBack.reportMotorTempuratures()[1],
            leftFront.reportMotorTempuratures()[0],
            leftFront.reportMotorTempuratures()[1],
            rightBack.reportMotorTempuratures()[0],
            rightBack.reportMotorTempuratures()[1],
            rightFront.reportMotorTempuratures()[0],
            rightFront.reportMotorTempuratures()[1],
        };
    }

    ////MOVEMENT FUNCTIONS
    
    //(f) -> Stops motors
    public void stopModules() {
        leftFront.stopMotors();
        leftBack.stopMotors();
        rightFront.stopMotors();
        rightBack.stopMotors();
    }

    //(f) -> Sets desired states of all modules in the Swerve Drive
    public void setDriveState(SwerveModuleState[] driveState) {
        SwerveDriveKinematics.desaturateWheelSpeeds(driveState, SwerveConstants.kMaxPhysicalSpeed); //Normailizing motor speeds
        //(i) Swerve Module States are in the format of an array with front 
        //(i) modules at [0,1] and back modules at [2,3], [left, right] respectively

        leftFront.setModuleState(driveState[0]);
        leftBack.setModuleState(driveState[2]);

        rightFront.setModuleState(driveState[1]);
        rightBack.setModuleState(driveState[3]);
    }

    //(f) -> Zeroes pivot encoders on the Swerve Modules
    public void zeroModulePivotPositions() {
        leftFront.zeroPivotEncoderToAbs();
        leftBack.zeroPivotEncoderToAbs();

        rightFront.zeroPivotEncoderToAbs();
        rightBack.zeroPivotEncoderToAbs();
    }
}