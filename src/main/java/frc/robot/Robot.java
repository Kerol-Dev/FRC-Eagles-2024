package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystem;

public class Robot extends TimedRobot{
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    // Robot başlatma fonksiyonu, robot başlatıldığında çağrılır
    m_robotContainer = new RobotContainer();
    DriveSubsystem.resetToAbsolute();
  }

  @Override
  public void robotPeriodic() {
    // Robot periyodik fonksiyonu, her döngüde çağrılır
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    // Otonom mod başlatıldığında çağrılır
    DriveSubsystem.resetEncoders();
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
  }

  @Override
  public void testInit() {
    // Test mod başlatıldığında çağrılır
    CommandScheduler.getInstance().cancelAll();
  }
}