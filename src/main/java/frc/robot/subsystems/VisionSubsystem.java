package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

public class VisionSubsystem extends SubsystemBase {
  private final DriveSubsystem swerve;
  public SwerveDrivePoseEstimator poseEst;

  public VisionSubsystem(DriveSubsystem swerve) {
    this.swerve = swerve;
    poseEst = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
        swerve.getHeading(),
        new SwerveModulePosition[] {
            DriveSubsystem.m_frontLeft.getPosition(),
            DriveSubsystem.m_frontRight.getPosition(),
            DriveSubsystem.m_rearLeft.getPosition(),
            DriveSubsystem.m_rearRight.getPosition() },
        new Pose2d());
  }

  public void periodic() {
    poseEst.update(swerve.getHeading(), new SwerveModulePosition[] {
        DriveSubsystem.m_frontLeft.getPosition(),
        DriveSubsystem.m_frontRight.getPosition(),
        DriveSubsystem.m_rearLeft.getPosition(),
        DriveSubsystem.m_rearRight.getPosition() });

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
    poseEst.resetPosition(swerve.getHeading(), new SwerveModulePosition[] {
        DriveSubsystem.m_frontLeft.getPosition(),
        DriveSubsystem.m_frontRight.getPosition(),
        DriveSubsystem.m_rearLeft.getPosition(),
        DriveSubsystem.m_rearRight.getPosition() }, newPose);
  }
}