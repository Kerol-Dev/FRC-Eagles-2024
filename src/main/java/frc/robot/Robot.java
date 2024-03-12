
package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    DriveSubsystem.resetToAbsolute();
    SmartDashboard.putNumber("Manual Shooter Angle", 0);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    if(DriverStation.isDisabled())
    {
      if(LimelightHelpers.getTV(""))
      {
        LimelightHelpers.setLEDMode_ForceBlink("");
        return;
      }
    }
    LimelightHelpers.setLEDMode_ForceOff("");
  }

  @Override
  public void autonomousInit() {
    DriveSubsystem.resetEncoders();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    m_robotContainer.driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
    m_robotContainer.m_ShooterSubsystem.setShooterAngleLocal();
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.m_ShooterSubsystem.stopShooterMotorsLocal();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}