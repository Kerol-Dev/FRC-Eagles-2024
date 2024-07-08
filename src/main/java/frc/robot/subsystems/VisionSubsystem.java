package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;

public class VisionSubsystem extends SubsystemBase {
  private final DriveSubsystem swerve; // Swerve sürüş sistemi
  public SwerveDrivePoseEstimator poseEst; // Swerve sürüş pozisyonu tahmin edici

  public VisionSubsystem(DriveSubsystem swerve) {
    this.swerve = swerve;

    poseEst = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
        swerve.getHeading(),
        new SwerveModulePosition[] {
            DriveSubsystem.m_frontLeft.getPosition(), // Ön sol modül pozisyonu
            DriveSubsystem.m_frontRight.getPosition(), // Ön sağ modül pozisyonu
            DriveSubsystem.m_rearLeft.getPosition(), // Arka sol modül pozisyonu
            DriveSubsystem.m_rearRight.getPosition() }, // Arka sağ modül pozisyonu
        new Pose2d()); // Başlangıç pozisyonu
  }

  public void periodic() {
    LimelightHelpers.SetRobotOrientation("", swerve.getHeading().getDegrees(), 0, 0, 0, 0, 0); // Robot yönünü ayarla

    poseEst.update(swerve.getHeading(), new SwerveModulePosition[] {
        DriveSubsystem.m_frontLeft.getPosition(),
        DriveSubsystem.m_frontRight.getPosition(),
        DriveSubsystem.m_rearLeft.getPosition(),
        DriveSubsystem.m_rearRight.getPosition() }); // Pozisyon tahmin ediciyi güncelle

    PoseEstimate blueRightBotPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(""); // Mavi takım pozisyon tahmini

    if (Math.abs(DriveSubsystem.m_gyro.getRate()) > 720)
      return; // Gyro hızı 720'den büyükse işlem yapma

    if (blueRightBotPose.rawFiducials.length > 0) {
      if (getAvgTA(blueRightBotPose.rawFiducials) > 0.0025) {
        System.out.println("aa");
        poseEst.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999)); // Görüş ölçüm standart sapmalarını ayarla
        poseEst.addVisionMeasurement(blueRightBotPose.pose, blueRightBotPose.timestampSeconds); // Görüş ölçümü ekle
      }
    }
  }

  public Pose2d getCurrentPose() {
    return poseEst.getEstimatedPosition(); // Mevcut pozisyonu al
  }

  public double getAvgTA(RawFiducial[] fiducials) {
    double sumTA = 0;
    for (int i = 0; i < fiducials.length; i++) {
      sumTA += fiducials[i].ta; // Fiducial TA değerlerini topla
    }
    return sumTA / fiducials.length; // Ortalama TA değerini döndür
  }

  public void setCurrentPose(Pose2d newPose) {
    poseEst.resetPosition(swerve.getHeading(), new SwerveModulePosition[] {
        DriveSubsystem.m_frontLeft.getPosition(),
        DriveSubsystem.m_frontRight.getPosition(),
        DriveSubsystem.m_rearLeft.getPosition(),
        DriveSubsystem.m_rearRight.getPosition() }, newPose); // Yeni pozisyonu ayarla
  }
}