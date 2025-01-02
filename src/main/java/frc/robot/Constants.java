package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
  public static final class DriveConstants {

    public static final double kMaxSpeedMetersPerSecond = 4.8; // Maksimum hız (m/s)
    public static final double kMaxAcceleration = 4; // Maksimum ivme
    public static final double kMaxAngularSpeed = 1 * Math.PI; // Maksimum açısal hız
    public static final double kMaxAngularAcceleration = 3; // Maksimum açısal ivme

    // Şasi konfigürasyonu
    public static final double kTrackWidth = 0.679; // Sağ ve sol tekerlekler arasındaki mesafe
    public static final double kWheelBase = 0.519; // Ön ve arka tekerlekler arasındaki mesafe

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

    public static final double kFrontLeftChassisAngularOffset = 0; // Ön sol şasi açısal ofseti
    public static final double kFrontRightChassisAngularOffset = 0; // Ön sağ şasi açısal ofseti
    public static final double kRearLeftChassisAngularOffset = 0; // Arka sol şasi açısal ofseti
    public static final double kRearRightChassisAngularOffset = 0; // Arka sağ şasi açısal ofseti

    // SPARK MAX CAN kimlikleri
    public static final int kFrontLeftDrivingCanId = 1; // Ön sol sürüş motoru CAN kimliği
    public static final int kFrontRightDrivingCanId = 2; // Ön sağ sürüş motoru CAN kimliği
    public static final int kRearLeftDrivingCanId = 3; // Arka sol sürüş motoru CAN kimliği
    public static final int kRearRightDrivingCanId = 4; // Arka sağ sürüş motoru CAN kimliği

    public static final int kFrontLeftTurningCanId = 14; // Ön sol dönüş motoru CAN kimliği
    public static final int kFrontRightTurningCanId = 16; // Ön sağ dönüş motoru CAN kimliği
    public static final int kRearLeftTurningCanId = 10; // Arka sol dönüş motoru CAN kimliği
    public static final int kRearRightTurningCanId = 15; // Arka sağ dönüş motoru CAN kimliği

    public static final int kFrontLeftcanCoderIDCanId = 20; // Ön sol canCoder ID CAN kimliği
    public static final int kFrontRightcanCoderIDCanId = 19; // Ön sağ canCoder ID CAN kimliği
    public static final int kRearLeftcanCoderIDCanId = 21; // Arka sol canCoder ID CAN kimliği
    public static final int kRearRightcanCoderIDCanId = 17; // Arka sağ canCoder ID CAN kimliği

    public static final float kFrontLeftcanCoderOffset = 46.75f; // Ön sol canCoder ofseti
    public static final float kFrontRightcanCoderOffset = 3.2f; // Ön sağ canCoder ofseti
    public static final float kRearLeftcanCoderOffset = -22.93f; // Arka sol canCoder ofseti
    public static final float kRearRightcanCoderOffset = 178.5f; // Arka sağ canCoder ofseti

    public static final boolean kGyroReversed = false; // Gyro yönü ters mi
    public static final double kGyroOffset = 0.0; // Gyro ofseti
  }

  public static final class ModuleConstants {

    public static final boolean kTurningEncoderInverted = true; // Dönüş enkoderi ters mi

    public static final double kWheelDiameterMeters = 0.103; // Tekerlek çapı (metre)
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI; // Tekerlek çevresi (metre)

    public static final double kDrivingMotorReduction = 6.5; // Sürüş motoru dişli oranı
    public static final double kTurningMotorReduction = 10.04; // Dönüş motoru dişli oranı

    public static final double kDrivingEncoderPositionFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction); // Sürüş enkoderi pozisyon faktörü (metre)
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // Sürüş enkoderi hız faktörü (metre/saniye)

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI) / kTurningMotorReduction; // Dönüş enkoderi pozisyon faktörü (radyan)
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / (60 * kTurningMotorReduction); // Dönüş enkoderi hız faktörü (radyan/saniye)

    public static final double kTurningEncoderPositionPIDMinInput = 0; // Dönüş enkoderi PID minimum giriş (radyan)
    public static final double kTurningEncoderPositionPIDMaxInput = Math.PI * 2; // Dönüş enkoderi PID maksimum giriş (radyan)

    public static final double kDrivingP = 0.2; // Sürüş motoru PID P parametresi
    public static final double kDrivingI = 0; // Sürüş motoru PID I parametresi
    public static final double kDrivingD = 0; // Sürüş motoru PID D parametresi
    public static final double kDrivingMinOutput = -1; // Sürüş motoru PID minimum çıkış
    public static final double kDrivingMaxOutput = 1; // Sürüş motoru PID maksimum çıkış

    public static final double kTurningP = 0.75; // Dönüş motoru PID P parametresi
    public static final double kTurningI = 0; // Dönüş motoru PID I parametresi
    public static final double kTurningD = 0; // Dönüş motoru PID D parametresi
    public static final double kTurningMinOutput = -1; // Dönüş motoru PID minimum çıkış
    public static final double kTurningMaxOutput = 1; // Dönüş motoru PID maksimum çıkış

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake; // Sürüş motoru boşta modu
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake; // Dönüş motoru boşta modu

    public static final int kDrivingMotorCurrentLimit = 40; // Sürüş motoru akım limiti (amper)
    public static final int kTurningMotorCurrentLimit = 40; // Dönüş motoru akım limiti (amper)
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0; // Sürücü kontrol portu
    public static final double kDriveDeadband = 0.1; // Sürüş ölü bandı
  }
}