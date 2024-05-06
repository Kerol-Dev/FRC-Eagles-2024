package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

@SuppressWarnings("removal")
public class SwerveModule {
  public final TalonFX m_drivingMotor;
  public final CANSparkMax m_turningSparkMax;

  public final RelativeEncoder m_turningEncoder;

  public final CANCoder m_canEncoder;

  public final SparkPIDController m_turningPIDController;

  private double m_chassisAngularOffset = 0;
  public double encoderOffset;
  private Rotation2d encoderOffset2d;

  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  @SuppressWarnings("deprecation")
  public SwerveModule(int drivingCANId, int turningCANId, int cancoderID, boolean drivingMotorReversed,
      boolean turningMotorReversed, double encoderOffset, boolean encoderInverted, double chassisAngularOffset) {

    m_chassisAngularOffset = chassisAngularOffset;
    encoderOffset2d = Rotation2d.fromDegrees(encoderOffset);
    this.encoderOffset = encoderOffset;

    m_drivingMotor = new TalonFX(drivingCANId);
    m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    m_canEncoder = new CANCoder(cancoderID);

    m_canEncoder.configSensorDirection(encoderInverted);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_turningSparkMax.restoreFactoryDefaults();

    m_turningSparkMax.setInverted(turningMotorReversed);

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_turningEncoder = m_turningSparkMax.getEncoder();
    m_turningPIDController = m_turningSparkMax.getPIDController();
    m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    m_turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the turning motor. Note these are example gains, and
    // you
    // may need to tune them for your own robot!
    m_turningPIDController.setP(ModuleConstants.kTurningP);
    m_turningPIDController.setI(ModuleConstants.kTurningI);
    m_turningPIDController.setD(ModuleConstants.kTurningD);
    m_turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
        ModuleConstants.kTurningMaxOutput);

    m_turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
    m_turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_turningSparkMax.burnFlash();

    // m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    // resetEncoders();

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(m_drivingMotor.getVelocity().getValueAsDouble() * 60, getAngle());
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getMotorPosition(), getAngle());
  }

  private Rotation2d getAngle() {
    return Rotation2d
        .fromRadians(m_turningEncoder.getPosition() - m_chassisAngularOffset);
  }

  private double getMotorPosition()
  {
    return m_drivingMotor.getPosition().getValueAsDouble() * ModuleConstants.kDrivingEncoderPositionFactor;
  }

  @SuppressWarnings("deprecation")
  public void updateSmartDashboard() {
    SmartDashboard.putNumber("Cancoder_" + m_canEncoder.getDeviceID(),
        getCanCoder().getDegrees());
    SmartDashboard.putNumber("Turning Angle" + m_canEncoder.getDeviceID(),
        Math.toDegrees((Math.abs(m_turningEncoder.getPosition()) % (2.0 * Math.PI))));
    SmartDashboard.putNumber("Driving Distance" + m_canEncoder.getDeviceID(), getMotorPosition());
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();

    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivingMotor.set(optimizedDesiredState.speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond);
    if (Math.abs(optimizedDesiredState.speedMetersPerSecond) < 0.006)
      m_drivingMotor.stopMotor();
    m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }

  public SwerveModuleState getDesiredState() {
    return m_desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingMotor.setPosition(0);
  }

  @SuppressWarnings("deprecation")
  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees((m_canEncoder.getAbsolutePosition()));
  }

  public void resetToAbsolute() {
    double absolutePosition = getCanCoder().getRadians() - encoderOffset2d.getRadians();
    m_turningEncoder.setPosition(absolutePosition);

    resetEncoders();
  }

  public void stop() {
    m_drivingMotor.set(0);
    m_turningSparkMax.set(0);
  }

  SwerveModuleState lastState;

  public static SwerveModuleState optimize(
      SwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees();
    if (Math.abs(delta) > 90) {
      targetSpeed = -targetSpeed;
      targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
    }
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }

  /**
   * @param scopeReference Current Angle
   * @param newAngle       Target Angle
   * @return Closest angle within scope
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
    return newAngle;
  }

}