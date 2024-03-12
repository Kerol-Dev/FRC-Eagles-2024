package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

public class VisionSubsystem extends SubsystemBase {
  private final DriveSubsystem swerve;
  public SwerveDrivePoseEstimator poseEst;

  public VisionSubsystem(DriveSubsystem swerve) {
    this.swerve = swerve;
    poseEst = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
        Rotation2d.fromDegrees(swerve.getHeading()),
        new SwerveModulePosition[] {
            DriveSubsystem.m_frontLeft.getPosition(),
            DriveSubsystem.m_frontRight.getPosition(),
            DriveSubsystem.m_rearLeft.getPosition(),
            DriveSubsystem.m_rearRight.getPosition() },
        new Pose2d());
  }

  public void periodic() {
    poseEst.update(Rotation2d.fromDegrees(swerve.getHeading()), new SwerveModulePosition[] {
        DriveSubsystem.m_frontLeft.getPosition(),
        DriveSubsystem.m_frontRight.getPosition(),
        DriveSubsystem.m_rearLeft.getPosition(),
        DriveSubsystem.m_rearRight.getPosition() });

    var blueRightResult = LimelightHelpers.getLatestResults("").targetingResults;

    Pose2d blueRightBotPose = blueRightResult.getBotPose2d_wpiBlue();

    double rightTimestamp = Timer.getFPGATimestamp() - (blueRightResult.latency_capture / 1000.0)
        - (blueRightResult.latency_pipeline / 1000.0);

    if (blueRightResult.targets_Fiducials.length > 0) {
      if (getAvgTA(blueRightResult.targets_Fiducials) > 0.0025) {
        poseEst.addVisionMeasurement(blueRightBotPose, rightTimestamp);
      }
    }

    SmartDashboard.putNumber("Limelight tX", LimelightHelpers.getTX(""));
    SmartDashboard.putNumber("Limelight 3D X", LimelightHelpers.getTargetPose3d_CameraSpace("").getX());
    SmartDashboard.putNumber("Limelight 3D Z", LimelightHelpers.getTargetPose3d_CameraSpace("").getZ());
  }

  public Pose2d getCurrentPose() {
    return poseEst.getEstimatedPosition();
  }

  public double getAvgTA(LimelightTarget_Fiducial[] fiducials) {
    double sumTA = 0;
    for (int i = 0; i < fiducials.length; i++) {
      sumTA += fiducials[i].ta;
    }
    return sumTA / fiducials.length;
  }

  public void setCurrentPose(Pose2d newPose) {
    poseEst.resetPosition(Rotation2d.fromDegrees(swerve.getHeading()), new SwerveModulePosition[] {
        DriveSubsystem.m_frontLeft.getPosition(),
        DriveSubsystem.m_frontRight.getPosition(),
        DriveSubsystem.m_rearLeft.getPosition(),
        DriveSubsystem.m_rearRight.getPosition() }, newPose);
  }
}