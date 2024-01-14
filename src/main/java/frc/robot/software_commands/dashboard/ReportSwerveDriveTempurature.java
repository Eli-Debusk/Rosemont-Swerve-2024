package frc.robot.software_commands.dashboard;

import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveDrive;
import frc.rosemont.util.DashboardUtils;

public class ReportSwerveDriveTempurature {
    
    private SwerveDrive subsystem;

    public ReportSwerveDriveTempurature(SwerveDrive subsystem) {
        this.subsystem = subsystem;
    }

    public void sendData() {
            DashboardUtils.reportTempuratureData(
            "MTEMP - LeftFrontDrive", subsystem.leftFront.getTempuratures()[0], SwerveConstants.kTempuratureWarningThreshold);

            DashboardUtils.reportTempuratureData(
            "MTEMP - LeftFrontPivot", subsystem.leftFront.getTempuratures()[1], SwerveConstants.kTempuratureWarningThreshold);

            DashboardUtils.reportTempuratureData(
            "MTEMP - LeftBackDrive", subsystem.leftBack.getTempuratures()[0], SwerveConstants.kTempuratureWarningThreshold);

            DashboardUtils.reportTempuratureData(
            "MTEMP - LeftBackPivot", subsystem.leftBack.getTempuratures()[1], SwerveConstants.kTempuratureWarningThreshold);

            DashboardUtils.reportTempuratureData(
            "MTEMP - RightFrontDrive", subsystem.rightFront.getTempuratures()[0], SwerveConstants.kTempuratureWarningThreshold);

            DashboardUtils.reportTempuratureData(
            "MTEMP - RightFrontPivot", subsystem.rightFront.getTempuratures()[1], SwerveConstants.kTempuratureWarningThreshold);

            DashboardUtils.reportTempuratureData(
            "MTEMP - RightBackDrive", subsystem.rightBack.getTempuratures()[0], SwerveConstants.kTempuratureWarningThreshold);

            DashboardUtils.reportTempuratureData(
            "MTEMP - RightBackPivot", subsystem.rightBack.getTempuratures()[1], SwerveConstants.kTempuratureWarningThreshold);

    }
}
