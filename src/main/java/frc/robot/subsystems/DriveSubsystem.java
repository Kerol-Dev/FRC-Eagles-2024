
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
  public static final SwerveModule m_frontRight = new SwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftcanCoderIDCanId,
      true,
      true,
      DriveConstants.kFrontLeftcanCoderOffset,
      false,
      DriveConstants.kFrontLeftChassisAngularOffset);

  public static final SwerveModule m_frontLeft = new SwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightcanCoderIDCanId,
      true,
      true,
      DriveConstants.kFrontRightcanCoderOffset,
      false,
      DriveConstants.kFrontRightChassisAngularOffset);

  public static final SwerveModule m_rearRight = new SwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kRearLeftcanCoderIDCanId,
      true,
      true,
      DriveConstants.kRearLeftcanCoderOffset,
      false,
      DriveConstants.kRearLeftChassisAngularOffset);

  public static final SwerveModule m_rearLeft = new SwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kRearRightcanCoderIDCanId,
      true,
      true,
      DriveConstants.kRearRightcanCoderOffset,
      false,
      DriveConstants.kRearRightChassisAngularOffset);

  public final Field2d m_field = new Field2d();
  private static AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  private SlewRateLimiter m_magXLimiter = new SlewRateLimiter(DriveConstants.kMaxAcceleration);
  private SlewRateLimiter m_magYLimiter = new SlewRateLimiter(DriveConstants.kMaxAcceleration);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kMaxAngularAcceleration);

  SwerveDriveOdometry odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
      getHeading(), new SwerveModulePosition[] {
          DriveSubsystem.m_frontLeft.getPosition(),
          DriveSubsystem.m_frontRight.getPosition(),
          DriveSubsystem.m_rearLeft.getPosition(),
          DriveSubsystem.m_rearRight.getPosition() });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetOdometry,
        this::getSpeeds,
        this::setSpeeds,
        new HolonomicPathFollowerConfig(
            new PIDConstants(1.5),
            new PIDConstants(3.5),
            4.5,
            0.427f,
            new ReplanningConfig()),
        () -> {
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue;
        }, this);
  }

  public ChassisSpeeds getSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState());
  }

  public void setSpeeds(ChassisSpeeds speeds) {
    speeds.omegaRadiansPerSecond *= -1;
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
    m_rearLeft.updateSmartDashboard();
    m_rearRight.updateSmartDashboard();
    m_frontLeft.updateSmartDashboard();
    m_frontRight.updateSmartDashboard();

    odometry.update(
        getHeading(), new SwerveModulePosition[] {
            DriveSubsystem.m_frontLeft.getPosition(),
            DriveSubsystem.m_frontRight.getPosition(),
            DriveSubsystem.m_rearLeft.getPosition(),
            DriveSubsystem.m_rearRight.getPosition() });

    SmartDashboard.putData(m_gyro);
    m_field.setRobotPose(odometry.getPoseMeters());
    SmartDashboard.putData(m_field);
  }

  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(m_gyro.getAngle());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public Field2d getField() {
    return m_field;
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
        getHeading(), new SwerveModulePosition[] {
            DriveSubsystem.m_frontLeft.getPosition(),
            DriveSubsystem.m_frontRight.getPosition(),
            DriveSubsystem.m_rearLeft.getPosition(),
            DriveSubsystem.m_rearRight.getPosition() },
        pose);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean robotCentric, boolean slowSpeed) {

    double xSpeedDelivered, ySpeedDelivered, rotDelivered;

    if (slowSpeed) {
      // Convert the commanded speeds into the correct units for the drivetrain
      xSpeedDelivered = m_magXLimiter.calculate(xSpeed * 0.3) * DriveConstants.kMaxSpeedMetersPerSecond;
      ySpeedDelivered = m_magYLimiter.calculate(ySpeed * 0.3) * DriveConstants.kMaxSpeedMetersPerSecond;
      rotDelivered = m_rotLimiter.calculate(rot * 0.3) * DriveConstants.kMaxAngularSpeed;

    } else {
      // Convert the commanded speeds into the correct units for the drivetrain
      xSpeedDelivered = m_magXLimiter.calculate(xSpeed) * DriveConstants.kMaxSpeedMetersPerSecond;
      ySpeedDelivered = m_magYLimiter.calculate(ySpeed) * DriveConstants.kMaxSpeedMetersPerSecond;
      rotDelivered = m_rotLimiter.calculate(rot) * DriveConstants.kMaxAngularSpeed;

    }

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        robotCentric
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
               getHeading())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    setModuleStates(swerveModuleStates);
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

  public Rotation2d getHeading() {
    return m_gyro.getRotation2d();
  }

  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public static void resetEncoders() {
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