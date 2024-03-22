package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  @SuppressWarnings("resource")
  @Override
  public void robotInit() {
    Logger.recordMetadata("Eagles 2024", "Main"); // Set a metadata value
    Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
    new PowerDistribution(0, ModuleType.kCTRE);

    Logger.start();

    m_robotContainer = new RobotContainer();
    DriveSubsystem.resetToAbsolute();
    SmartDashboard.putNumber("Manual Shooter Angle", 0);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    if (DriverStation.isAutonomousEnabled()) {
      LimelightHelpers.setLEDMode_ForceOff("");
      return;
    }

    if (SmartDashboard.getNumber("Manual Shooter Angle", 0) > 0) {
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
    m_robotContainer.m_ClimbSubsystem.climbMotor.getEncoder().setPosition(Preferences.getDouble("ClimbPos", 0));
  }

  @Override
  public void autonomousInit() {
    DriveSubsystem.resetEncoders();
    ShooterSubsystem.shooterMotorHinge.getEncoder().setPosition(0);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    RobotContainer.driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.m_ShooterSubsystem.stopShooterMotorsLocal();
  }

  @Override
  public void teleopExit() {
      Preferences.setDouble("ClimbPos", m_robotContainer.m_ClimbSubsystem.climbMotor.getEncoder().getPosition());
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}