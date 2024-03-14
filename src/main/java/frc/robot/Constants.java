package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {
  public static final class DriveConstants {

    public static final double kMaxSpeedMetersPerSecond = 4.5;
    public static final double kMaxAcceleration = 2.0;
    public static final double kMaxAngularSpeed = 2 * Math.PI;
    public static final double kMaxAngularAcceleration = 2;

    // Chassis configuration
    public static final double kTrackWidth = 0.679;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.519;
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2));
   
    public static final double kFrontLeftChassisAngularOffset = 0;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kRearLeftChassisAngularOffset = 0;
    public static final double kRearRightChassisAngularOffset = 0;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kFrontRightDrivingCanId = 2;
    public static final int kRearLeftDrivingCanId = 3;
    public static final int kRearRightDrivingCanId = 4;

    public static final int kFrontLeftTurningCanId = 5;
    public static final int kFrontRightTurningCanId = 6;
    public static final int kRearLeftTurningCanId = 7;
    public static final int kRearRightTurningCanId = 8;

    public static final int kFrontLeftcanCoderIDCanId = 17;
    public static final int kFrontRightcanCoderIDCanId = 18;
    public static final int kRearLeftcanCoderIDCanId = 19;
    public static final int kRearRightcanCoderIDCanId = 20;

    public static final float kFrontLeftcanCoderOffset = 359.5f;
    public static final float kFrontRightcanCoderOffset = 204.4f;
    public static final float kRearLeftcanCoderOffset = 176.4f;
    public static final float kRearRightcanCoderOffset = 131.6f;

    public static final boolean kGyroReversed = false;
    public static final double kGyroOffset = 180.0;
  }

  public static final class ModuleConstants {

    public static final boolean kTurningEncoderInverted = true;

    public static final double kWheelDiameterMeters = 0.1;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    
    public static final double kDrivingMotorReduction = 6.5;
    public static final double kTurningMotorReduction = 10.0446;

    public static final double kDrivingEncoderPositionFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction); // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI) / kTurningMotorReduction; // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) /(60 * kTurningMotorReduction); // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = Math.PI *2; // radians

    public static final double kDrivingP = 0.18;		// 0.025
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 0.5;  // 0.03
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 40; // amps
    public static final int kTurningMotorCurrentLimit = 40; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }
}