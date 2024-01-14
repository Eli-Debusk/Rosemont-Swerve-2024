package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.software_commands.dashboard.ReportSwerveDriveTempurature;
import frc.rosemont.util.RoboMath;
import frc.rosemont.util.RosemontConstants.SwerveModulePositions;
import frc.rosemont.util.profiles.DefaultSwerveModuleProfile;

public class SwerveDrive extends SubsystemBase {

    ////VARIABLE CONSTRUCTION

    //(i) Swerve Modules (given a SwerveModuleProfile)
    public final SwerveModule leftBack = new SwerveModule(new DefaultSwerveModuleProfile(SwerveModulePositions.LEFTBACK));
    public final SwerveModule leftFront = new SwerveModule(new DefaultSwerveModuleProfile(SwerveModulePositions.LEFTFRONT));
    public final SwerveModule rightBack = new SwerveModule(new DefaultSwerveModuleProfile(SwerveModulePositions.RIGHTBACK));
    public final SwerveModule rightFront = new SwerveModule(new DefaultSwerveModuleProfile(SwerveModulePositions.RIGHTFRONT));

    //(i) KuaiLabs NavX Gyroscope
    private final AHRS gyroscope = new AHRS(Port.kMXP);

    //(i) FieldData and OdometryData
    private final Field2d fieldData = new Field2d();
    private final SwerveDriveOdometry odometryData = new SwerveDriveOdometry(
        SwerveConstants.swerveKinematics, getRotation2D(), getModulePositions());

    //(i) Tempurature Reporting
    private ReportSwerveDriveTempurature tempuratureReporter = new ReportSwerveDriveTempurature(this);
    
    ////CLASS STRUCTURE
    
    public SwerveDrive() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);

                resetHeading(); //(f) Resets gyroscope angle
                SmartDashboard.putData("Field Data", fieldData); //(f) Sends field data to NetworkTables

            } catch (Exception e) {}
        })
        .start();
    }

    @Override
    public void periodic() {
        odometryData.update(getRotation2D(), getModulePositions()); //(i) Updates odometry with new pose construction data
        fieldData.setRobotPose(odometryData.getPoseMeters()); //(i) Updates field data with odometry pose

        //(s) Tempurature Data -> NetworkTables
        tempuratureReporter.sendData();
    }

    ////FEEDBACK FUNCTIONS

    //(s) Navigation Feedback

    public double getHeading() {
        return RoboMath.headingRemainder(gyroscope.getAngle()); //(i) Returns robot heading
    }

    public Rotation2d getRotation2D() {
        return Rotation2d.fromDegrees(getHeading()); //(i) Returns robot heading in Rotation2d format
    }

    public SwerveModulePosition[] getModulePositions() {
       //(i) Constructs SwerveModulePosition array with all four module positions
        return new SwerveModulePosition[] {
            leftFront.getModulePosition(),
            rightFront.getModulePosition(),
            leftBack.getModulePosition(),
            rightBack.getModulePosition()
        };
    }

    ////MOVEMENT FUNCTIONS
    
    //(f) Stops motors for Safety and Utility reasons
    public void stopModules() {
        leftFront.stop();
        leftBack.stop();
        rightFront.stop();
        rightBack.stop();
    }

    //(f) Sets desired states of all modules in the Swerve Drive
    public void setDesiredChassisSpeeds(ChassisSpeeds desiredChassisSpeeds) {
        SwerveModuleState[] desiredDriveState = SwerveConstants.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(desiredDriveState, SwerveConstants.kMaxPhysicalSpeed);

        leftFront.setDesiredModuleState(desiredDriveState[0]);
        leftBack.setDesiredModuleState(desiredDriveState[2]);

        rightFront.setDesiredModuleState(desiredDriveState[1]);
        rightBack.setDesiredModuleState(desiredDriveState[3]);
    }

    ////UTIL FUNCTIONS

    public void resetHeading() {
        gyroscope.reset(); //(i) Resets gyroscope yaw
    }

    //(f) Currently Unused
    public void resetModules() {
        leftFront.resetEncoderPositions();
        leftBack.resetEncoderPositions();

        rightFront.resetEncoderPositions();
        rightBack.resetEncoderPositions();
    }

    //(f) Autosets pivot positions to ensure autozeroing
    public void autoZeroModulePositions() {
        leftFront.autoZeroPivotEncoder();
        rightBack.autoZeroPivotEncoder();

        leftBack.autoZeroPivotEncoder();
        rightBack.autoZeroPivotEncoder();
    }
}