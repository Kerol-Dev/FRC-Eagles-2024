package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // MAXSwerveModules oluştur
  public static final SwerveModule m_frontRight = new SwerveModule(
      DriveConstants.kFrontLeftDrivingCanId, // Ön sol sürüş CAN ID
      DriveConstants.kFrontLeftTurningCanId, // Ön sol dönüş CAN ID
      DriveConstants.kFrontLeftcanCoderIDCanId, // Ön sol canCoder ID
      true, // Ters
      true, // Encoder ters
      DriveConstants.kFrontLeftcanCoderOffset, // Ön sol canCoder ofseti
      false, // Fren modu
      DriveConstants.kFrontLeftChassisAngularOffset); // Ön sol şasi açısal ofseti

  public static final SwerveModule m_frontLeft = new SwerveModule(
      DriveConstants.kFrontRightDrivingCanId, // Ön sağ sürüş CAN ID
      DriveConstants.kFrontRightTurningCanId, // Ön sağ dönüş CAN ID
      DriveConstants.kFrontRightcanCoderIDCanId, // Ön sağ canCoder ID
      false, // Ters
      true, // Encoder ters
      DriveConstants.kFrontRightcanCoderOffset, // Ön sağ canCoder ofseti
      false, // Fren modu
      DriveConstants.kFrontRightChassisAngularOffset); // Ön sağ şasi açısal ofseti

  public static final SwerveModule m_rearRight = new SwerveModule(
      DriveConstants.kRearLeftDrivingCanId, // Arka sol sürüş CAN ID
      DriveConstants.kRearLeftTurningCanId, // Arka sol dönüş CAN ID
      DriveConstants.kRearLeftcanCoderIDCanId, // Arka sol canCoder ID
      false, // Ters
      true, // Encoder ters
      DriveConstants.kRearLeftcanCoderOffset, // Arka sol canCoder ofseti
      false, // Fren modu
      DriveConstants.kRearLeftChassisAngularOffset); // Arka sol şasi açısal ofseti

  public static final SwerveModule m_rearLeft = new SwerveModule(
      DriveConstants.kRearRightDrivingCanId, // Arka sağ sürüş CAN ID
      DriveConstants.kRearRightTurningCanId, // Arka sağ dönüş CAN ID
      DriveConstants.kRearRightcanCoderIDCanId, // Arka sağ canCoder ID
      true, // Ters
      true, // Encoder ters
      DriveConstants.kRearRightcanCoderOffset, // Arka sağ canCoder ofseti
      false, // Fren modu
      DriveConstants.kRearRightChassisAngularOffset); // Arka sağ şasi açısal ofseti

  public final Field2d m_field = new Field2d(); // Alan bilgisi
  public static AHRS m_gyro = new AHRS(SPI.Port.kMXP); // Gyro sensörü

  private SlewRateLimiter m_magXLimiter = new SlewRateLimiter(DriveConstants.kMaxAcceleration); // X ekseni ivme sınırlayıcı
  private SlewRateLimiter m_magYLimiter = new SlewRateLimiter(DriveConstants.kMaxAcceleration); // Y ekseni ivme sınırlayıcı
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kMaxAngularAcceleration); // Dönüş ivme sınırlayıcı

  private VisionSubsystem visionSubsystem = new VisionSubsystem(this); // Görüş sistemi

  /** Yeni bir DriveSubsystem oluşturur. */
  public DriveSubsystem() {
    AutoBuilder.configureHolonomic(
        this::getPose, // Pozisyonu al
        this::resetOdometry, // Odometrini sıfırla
        this::getSpeeds, // Hızları al
        this::setSpeeds, // Hızları ayarla
        new HolonomicPathFollowerConfig(
            new PIDConstants(2), // PID P parametresi
            new PIDConstants(3), // PID I parametresi
            4.5, // PID D parametresi
            0.427f, // Maksimum hız
            new ReplanningConfig()), // Yeniden planlama yapılandırması
        () -> {
          var alliance = DriverStation.getAlliance(); // Takım bilgisi
          return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue; // Takım mavi mi
        }, this);
  }

  public ChassisSpeeds getSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(), // Ön sol modül durumu
        m_frontRight.getState(), // Ön sağ modül durumu
        m_rearLeft.getState(), // Arka sol modül durumu
        m_rearRight.getState()); // Arka sağ modül durumu
  }

  public void setSpeeds(ChassisSpeeds speeds) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds); // Swerve modül durumlarını al
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond); // Tekerlek hızlarını ayarla
    m_frontLeft.setDesiredState(swerveModuleStates[0]); // Ön sol durumu ayarla
    m_frontRight.setDesiredState(swerveModuleStates[1]); // Ön sağ durumu ayarla
    m_rearLeft.setDesiredState(swerveModuleStates[2]); // Arka sol durumu ayarla
    m_rearRight.setDesiredState(swerveModuleStates[3]); // Arka sağ durumu ayarla
  }

  @Override
  public void periodic() {
    // Periodik olarak odometriyi güncelle
    m_rearLeft.updateSmartDashboard(); // Arka sol modül bilgilerini güncelle
    m_rearRight.updateSmartDashboard(); // Arka sağ modül bilgilerini güncelle
    m_frontLeft.updateSmartDashboard(); // Ön sol modül bilgilerini güncelle
    m_frontRight.updateSmartDashboard(); // Ön sağ modül bilgilerini güncelle

    SmartDashboard.putData(m_gyro); // Gyro bilgilerini SmartDashboard'a yazdır
    m_field.setRobotPose(visionSubsystem.poseEst.getEstimatedPosition()); // Robot pozisyonunu alana yazdır
    SmartDashboard.putData(m_field); // Alan bilgilerini SmartDashboard'a yazdır
  }

  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(m_gyro.getAngle()); // Yaw (dönüş) açısını al
  }

  public Pose2d getPose() {
    return visionSubsystem.poseEst.getEstimatedPosition(); // Robot pozisyonunu al
  }

  public Field2d getField() {
    return m_field; // Alan bilgilerini al
  }

  public void resetOdometry(Pose2d pose) {
    visionSubsystem.poseEst.resetPosition(
        getHeading(), new SwerveModulePosition[] {
            DriveSubsystem.m_frontLeft.getPosition(), // Ön sol modül pozisyonu
            DriveSubsystem.m_frontRight.getPosition(), // Ön sağ modül pozisyonu
            DriveSubsystem.m_rearLeft.getPosition(), // Arka sol modül pozisyonu
            DriveSubsystem.m_rearRight.getPosition() }, // Arka sağ modül pozisyonu
        pose); // Pozisyonu sıfırla
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean robotCentric, boolean slowSpeed) {

    double xSpeedDelivered, ySpeedDelivered, rotDelivered;

    if (slowSpeed) {
      // Komut hızlarını sürüş sistemine doğru birimlere dönüştür
      xSpeedDelivered = m_magXLimiter.calculate(xSpeed * 0.3) * DriveConstants.kMaxSpeedMetersPerSecond;
      ySpeedDelivered = m_magYLimiter.calculate(ySpeed * 0.3) * DriveConstants.kMaxSpeedMetersPerSecond;
      rotDelivered = m_rotLimiter.calculate(rot * 0.3) * DriveConstants.kMaxAngularSpeed;
    } else {
      // Komut hızlarını sürüş sistemine doğru birimlere dönüştür
      xSpeedDelivered = m_magXLimiter.calculate(xSpeed) * DriveConstants.kMaxSpeedMetersPerSecond;
      ySpeedDelivered = m_magYLimiter.calculate(ySpeed) * DriveConstants.kMaxSpeedMetersPerSecond;
      rotDelivered = m_rotLimiter.calculate(rot) * DriveConstants.kMaxAngularSpeed;
    }

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        robotCentric
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, getHeading())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    setModuleStates(swerveModuleStates); // Modül durumlarını ayarla
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond); // Tekerlek hızlarını ayarla
    m_frontLeft.setDesiredState(desiredStates[0]); // Ön sol durumu ayarla
    m_frontRight.setDesiredState(desiredStates[1]); // Ön sağ durumu ayarla
    m_rearLeft.setDesiredState(desiredStates[2]); // Arka sol durumu ayarla
    m_rearRight.setDesiredState(desiredStates[3]); // Arka sağ durumu ayarla
  }

  public void zeroHeading() {
    m_gyro.reset(); // Gyro başlığını sıfırla
  }

  public Rotation2d getHeading() {
    return m_gyro.getRotation2d(); // Gyro dönüş açısını al
  }

  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0); // Gyro dönüş hızını al
  }

  public static void resetEncoders() {
    m_frontLeft.resetEncoders(); // Ön sol enkoderleri sıfırla
    m_rearLeft.resetEncoders(); // Arka sol enkoderleri sıfırla
    m_frontRight.resetEncoders(); // Ön sağ enkoderleri sıfırla
    m_rearRight.resetEncoders(); // Arka sağ enkoderleri sıfırla
    m_gyro.reset(); // Gyro'yu sıfırla
  }

  public static void resetToAbsolute() {
    m_frontLeft.resetToAbsolute(); // Ön sol mutlak pozisyonu sıfırla
    m_frontRight.resetToAbsolute(); // Ön sağ mutlak pozisyonu sıfırla
    m_rearLeft.resetToAbsolute(); // Arka sol mutlak pozisyonu sıfırla
    m_rearRight.resetToAbsolute(); // Arka sağ mutlak pozisyonu sıfırla
  }

  public void stop() {
    m_rearLeft.stop(); // Arka sol modülü durdur
    m_frontLeft.stop(); // Ön sol modülü durdur
    m_rearRight.stop(); // Arka sağ modülü durdur
    m_frontRight.stop(); // Ön sağ modülü durdur
  }
}