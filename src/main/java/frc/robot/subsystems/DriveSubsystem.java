
package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
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
  // Create MAXSwerveModules
  private static final SwerveModule m_frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftcanCoderIDCanId,
      false,
      false,
      DriveConstants.kFrontLeftcanCoderOffset,
      false,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private static final SwerveModule m_frontRight = new SwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightcanCoderIDCanId,
      false,
      false,
      DriveConstants.kFrontRightcanCoderOffset,
      false,
      DriveConstants.kFrontRightChassisAngularOffset);

  private static final SwerveModule m_rearLeft = new SwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kRearLeftcanCoderIDCanId,
      false,
      false,
      DriveConstants.kRearLeftcanCoderOffset,
      false,
      DriveConstants.kRearLeftChassisAngularOffset);

  private static final SwerveModule m_rearRight = new SwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kRearRightcanCoderIDCanId,
      false,
      false,
      DriveConstants.kRearRightcanCoderOffset,
      false,
      DriveConstants.kRearRightChassisAngularOffset);

  public final Field2d m_field = new Field2d();
  private AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  private SlewRateLimiter m_magXLimiter = new SlewRateLimiter(DriveConstants.kMaxAcceleration);
  private SlewRateLimiter m_magYLimiter = new SlewRateLimiter(DriveConstants.kMaxAcceleration);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kMaxAngularAcceleration);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetOdometry,
        this::getSpeeds,
        this::setSpeeds,
        new HolonomicPathFollowerConfig(
            new PIDConstants(4.5),
            new PIDConstants(2),
            4.5,
            0.4030f,
            new ReplanningConfig()),
        () -> {
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
        }
        , this);
  }

  public ChassisSpeeds getSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
  }

  public void setSpeeds(ChassisSpeeds speeds) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    m_rearLeft.updateSmartDashboard();
    m_rearRight.updateSmartDashboard();
    m_frontLeft.updateSmartDashboard();
    m_frontRight.updateSmartDashboard();

    SmartDashboard.putData(m_gyro);
    SmartDashboard.putBoolean("Gyro Connection", m_gyro.isConnected());

    m_field.setRobotPose(m_odometry.getPoseMeters());

    SmartDashboard.putNumber("Robot Heading", m_gyro.getAngle());
    SmartDashboard.putString("Robot Location", getPose().getRotation().toString());
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    SmartDashboard.putData(m_field);
  }

  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(m_gyro.getAngle());
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public Field2d getField() {
    return m_field;
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean robotCentric, boolean slowSpeed) {

    double xSpeedDelivered, ySpeedDelivered, rotDelivered;

    if (slowSpeed) {
      // Convert the commanded speeds into the correct units for the drivetrain
      xSpeedDelivered = m_magXLimiter.calculate(xSpeed * 0.2) * DriveConstants.kMaxSpeedMetersPerSecond;
      ySpeedDelivered = m_magYLimiter.calculate(ySpeed * 0.2) * DriveConstants.kMaxSpeedMetersPerSecond;
      rotDelivered = m_rotLimiter.calculate(rot * 0.2) * DriveConstants.kMaxAngularSpeed;

    } else {
      // Convert the commanded speeds into the correct units for the drivetrain
      xSpeedDelivered = m_magXLimiter.calculate(xSpeed) * DriveConstants.kMaxSpeedMetersPerSecond;
      ySpeedDelivered = m_magYLimiter.calculate(ySpeed) * DriveConstants.kMaxSpeedMetersPerSecond;
      rotDelivered = m_rotLimiter.calculate(rot) * DriveConstants.kMaxAngularSpeed;

    }

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        robotCentric
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(m_gyro.getYaw()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  public double getHeading() {
    return m_gyro.getAngle();
  }

  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
    m_gyro.reset();
  }

  public static void resetToAbsolute() {
    m_frontLeft.resetToAbsolute();
    m_frontRight.resetToAbsolute();
    m_rearLeft.resetToAbsolute();
    m_rearRight.resetToAbsolute();
  }

  public void stop() {
    m_rearLeft.stop();
    m_frontLeft.stop();
    m_rearRight.stop();
    m_frontRight.stop();
  }
}