package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Robot extends TimedRobot{
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    // Robot başlatma fonksiyonu, robot başlatıldığında çağrılır
    m_robotContainer = new RobotContainer();
    DriveSubsystem.resetToAbsolute();
    SmartDashboard.putNumber("Manual Shooter Angles", 0);
  }

  @Override
  public void robotPeriodic() {
    // Robot periyodik fonksiyonu, her döngüde çağrılır
    CommandScheduler.getInstance().run();

    if (DriverStation.isAutonomousEnabled()) {
      LimelightHelpers.setLEDMode_ForceOff("");
      return;
    }

    if (SmartDashboard.getNumber("Manual Shooter Angles", 0) > 0) {
      LimelightHelpers.setLEDMode_ForceOff("");
      return;
    }

    if (DriverStation.isDisabled()) {
      if (LimelightHelpers.getTV("")) {
        LimelightHelpers.setLEDMode_ForceOff("");
      } else
        LimelightHelpers.setLEDMode_ForceBlink("");
    } else if (LimelightHelpers.isPossible())
      LimelightHelpers.setLEDMode_ForceOff("");
    else
      LimelightHelpers.setLEDMode_ForceOn("");
  }

  @Override
  public void disabledInit() {
    // Robot devre dışı bırakıldığında çağrılır
    m_robotContainer.m_ClimbSubsystem.climbMotor.getEncoder().setPosition(Preferences.getDouble("ClimbPos", 0));
  }

  @Override
  public void autonomousInit() {
    // Otonom mod başlatıldığında çağrılır
    DriveSubsystem.resetEncoders();
    ShooterSubsystem.shooterMotorHinge.getEncoder().setPosition(0);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    // Otonom mod sırasında her döngüde çağrılır
    RobotContainer.driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
  }

  @Override
  public void teleopInit() {
    // Teleop mod başlatıldığında çağrılır
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.m_ShooterSubsystem.stopShooterMotorsLocal();
  }

  @Override
  public void teleopExit() {
    // Teleop mod sona erdiğinde çağrılır
    Preferences.setDouble("ClimbPos", m_robotContainer.m_ClimbSubsystem.climbMotor.getEncoder().getPosition());
  }

  @Override
  public void testInit() {
    // Test mod başlatıldığında çağrılır
    CommandScheduler.getInstance().cancelAll();
  }
}