package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

@SuppressWarnings("removal")
public class SwerveModule {
  public final TalonFX m_drivingMotor; // Sürüş motoru
  public final CANSparkMax m_turningSparkMax; // Dönüş motoru

  public final RelativeEncoder m_turningEncoder; // Dönüş enkoderi

  public final CANCoder m_canEncoder; // CAN enkoder

  public final SparkPIDController m_turningPIDController; // Dönüş PID kontrolcüsü

  private double m_chassisAngularOffset = 0; // Şasi açısal ofseti
  public double encoderOffset; // Enkoder ofseti
  private Rotation2d encoderOffset2d; // Enkoder ofseti (Rotation2d)

  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d()); // İstenen durum

  public SwerveModule(int drivingCANId, int turningCANId, int cancoderID, boolean drivingMotorReversed,
      boolean turningMotorReversed, double encoderOffset, boolean encoderInverted, double chassisAngularOffset) {

    m_chassisAngularOffset = chassisAngularOffset; // Şasi açısal ofsetini ayarlama
    encoderOffset2d = Rotation2d.fromDegrees(encoderOffset); // Enkoder ofsetini ayarlama
    this.encoderOffset = encoderOffset;

    m_drivingMotor = new TalonFX(drivingCANId); // Sürüş motorunu başlatma
    m_drivingMotor.setInverted(drivingMotorReversed); // Sürüş motoru yönünü ayarlama
    m_drivingMotor.setNeutralMode(NeutralModeValue.Brake);
    CurrentLimitsConfigs cl = new CurrentLimitsConfigs();
    cl.SupplyCurrentLimitEnable = true;
    cl.StatorCurrentLimitEnable = true;
    cl.SupplyCurrentLimit = 100;
    cl.StatorCurrentLimit = 80;
    m_drivingMotor.getConfigurator().apply(cl);
    m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless); // Dönüş motorunu başlatma

    m_canEncoder = new CANCoder(cancoderID); // CAN enkoderi başlatma

    m_canEncoder.configSensorDirection(encoderInverted); // Enkoder yönünü ayarlama

    // SPARK MAX'i fabrika ayarlarına sıfırlama
    m_turningSparkMax.restoreFactoryDefaults();

    m_turningSparkMax.setInverted(turningMotorReversed); // Dönüş motoru yönünü ayarlama

    // Enkoderleri ve PID kontrolcülerini başlatma
    m_turningEncoder = m_turningSparkMax.getEncoder();
    m_turningPIDController = m_turningSparkMax.getPIDController();
    m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    // Enkoder konum ve hız dönüştürme faktörlerini ayarlama
    m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

    // Dönüş enkoderini ters çevirme
    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    m_turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    // Dönüş motoru için PID kazançlarını ayarlama
    m_turningPIDController.setP(ModuleConstants.kTurningP);
    m_turningPIDController.setI(ModuleConstants.kTurningI);
    m_turningPIDController.setD(ModuleConstants.kTurningD);
    m_turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput);

    m_turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode); // Boşta modunu ayarlama
    m_turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit); // Akım sınırını ayarlama

    // SPARK MAX yapılandırmalarını kaydetme
    m_turningSparkMax.burnFlash();

    m_chassisAngularOffset = chassisAngularOffset; // Şasi açısal ofsetini ayarlama
    m_desiredState.angle = new Rotation2d(0); // İstenen durumu ayarlama
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(m_drivingMotor.getVelocity().getValueAsDouble() * 60, getAngle()); // Modül durumunu alma
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getMotorPosition(), getAngle()); // Modül pozisyonunu alma
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromRadians(m_turningEncoder.getPosition() - m_chassisAngularOffset); // Modül açısını alma
  }

  private double getMotorPosition() {
    return m_drivingMotor.getPosition().getValueAsDouble() * ModuleConstants.kDrivingEncoderPositionFactor; // Motor pozisyonunu alma
  }

  public void updateSmartDashboard() {
    SmartDashboard.putNumber("Cancoder_" + m_canEncoder.getDeviceID(), getCanCoder().getDegrees()); // SmartDashboard'a CAN enkoder verilerini yazdırma
    SmartDashboard.putNumber("Turning Angle" + m_canEncoder.getDeviceID(),
        Math.toDegrees((Math.abs(m_turningEncoder.getPosition()) % (2.0 * Math.PI)))); // SmartDashboard'a dönüş açısını yazdırma
    SmartDashboard.putNumber("Driving Distance" + m_canEncoder.getDeviceID(), getMotorPosition()); // SmartDashboard'a sürüş mesafesini yazdırma
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // İstenen duruma şasi açısal ofsetini uygula
    SwerveModuleState correctedDesiredState = new SwerveModuleState();

    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // İstenen durumu optimize et
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(m_turningEncoder.getPosition()));

    // Sürüş ve dönüş motorlarını hedef duruma yönlendir
    m_drivingMotor.set(optimizedDesiredState.speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond);
    if (Math.abs(optimizedDesiredState.speedMetersPerSecond) < 0.006)
      m_drivingMotor.stopMotor();
    m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    m_desiredState = desiredState; // İstenen durumu ayarla
  }

  public SwerveModuleState getDesiredState() {
    return m_desiredState; // İstenen durumu döndür
  }

  /** Tüm SwerveModule enkoderlerini sıfırlar. */
  public void resetEncoders() {
    m_drivingMotor.setPosition(0); // Enkoderleri sıfırla
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees((m_canEncoder.getAbsolutePosition())); // CAN enkoder pozisyonunu döndür
  }

  public void resetToAbsolute() {
    double absolutePosition = getCanCoder().getRadians() - encoderOffset2d.getRadians(); // Mutlak pozisyonu hesapla
    m_turningEncoder.setPosition(absolutePosition); // Enkoder pozisyonunu ayarla

    resetEncoders(); // Enkoderleri sıfırla
  }

  public void stop() {
    m_drivingMotor.set(0); // Sürüş motorunu durdur
    m_turningSparkMax.set(0); // Dönüş motorunu durdur
  }

  SwerveModuleState lastState;

  public static SwerveModuleState optimize(
      SwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees()); // Hedef açıyı belirle
    double targetSpeed = desiredState.speedMetersPerSecond; // Hedef hızı belirle
    double delta = targetAngle - currentAngle.getDegrees();
    if (Math.abs(delta) > 90) {
      targetSpeed = -targetSpeed;
      targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180); // Hedef açıyı optimize et
    }
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle)); // İstenen durumu döndür
  }

  /**
   * @param scopeReference Mevcut Açı
   * @param newAngle       Hedef Açı
   * @return Yakın açıyı döndür
   */

  private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
    double lowerBound;
    double upperBound;
    double lowerOffset = scopeReference % 360;
    if (lowerOffset >= 0) {
      lowerBound = scopeReference - lowerOffset;
      upperBound = scopeReference + (360 - lowerOffset);
    } else {
      upperBound = scopeReference - lowerOffset;
      lowerBound = scopeReference - (360 + lowerOffset);
    }
    while (newAngle < lowerBound) {
      newAngle += 360;
    }
    while (newAngle > upperBound) {
      newAngle -= 360;
    }
    if (newAngle - scopeReference > 180) {
      newAngle -= 360;
    } else if (newAngle - scopeReference < -180) {
      newAngle += 360;
    }
    return newAngle; // En yakın açıyı döndür
  }

}