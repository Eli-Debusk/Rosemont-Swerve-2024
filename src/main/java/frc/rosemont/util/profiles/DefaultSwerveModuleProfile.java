package frc.rosemont.util.profiles;

import frc.robot.Constants.SwerveConstants;
import frc.rosemont.util.RosemontConstants.*;

public class DefaultSwerveModuleProfile {  
     
    public final int driveCID;
    public final int pivotCID;
    public final int absEncoderCID;
    public final boolean driveReversed;
    public final boolean pivotReversed;

    public DefaultSwerveModuleProfile(int modulePosition) {

        if (modulePosition == SwerveModulePositions.LEFTBACK) {
            driveCID = SwerveConstants.kLeftBackDriveCAN_ID;
            pivotCID = SwerveConstants.kLeftBackPivotCAN_ID;
            absEncoderCID = SwerveConstants.kLeftBackAbsoluteEncoderCAN_ID;
            driveReversed = SwerveConstants.kLeftBackDirections[0];
            pivotReversed = SwerveConstants.kLeftBackDirections[1];

        } else if (modulePosition == SwerveModulePositions.LEFTFRONT) {
            driveCID = SwerveConstants.kLeftFrontDriveCAN_ID;
            pivotCID = SwerveConstants.kLeftFrontPivotCAN_ID;
            absEncoderCID = SwerveConstants.kLeftFrontAbsoluteEncoderCAN_ID;
            driveReversed = SwerveConstants.kLeftFrontDirections[0];
            pivotReversed = SwerveConstants.kLeftFrontDirections[1];

        } else if (modulePosition == SwerveModulePositions.RIGHTBACK) {
            driveCID = SwerveConstants.kRightBackDriveCAN_ID;
            pivotCID = SwerveConstants.kRightBackPivotCAN_ID;
            absEncoderCID = SwerveConstants.kRightBackAbsoluteEncoderCAN_ID;
            driveReversed = SwerveConstants.kRightBackDirections[0];
            pivotReversed = SwerveConstants.kRightBackDirections[1];

        } else if(modulePosition == SwerveModulePositions.RIGHTFRONT) {
            driveCID = SwerveConstants.kRightFrontDriveCAN_ID;
            pivotCID = SwerveConstants.kRightFrontPivotCAN_ID;
            absEncoderCID = SwerveConstants.kRightFrontAbsoluteEncoderCAN_ID;
            driveReversed = SwerveConstants.kRightFrontDirections[0];
            pivotReversed = SwerveConstants.kRightFrontDirections[1];
            
        } else {
            driveCID = 0;
            pivotCID = 0;
            absEncoderCID = 0;
            driveReversed = false;
            pivotReversed = false;
        }

    }
}
