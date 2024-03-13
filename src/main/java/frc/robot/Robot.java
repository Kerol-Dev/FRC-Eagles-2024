package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystem;

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