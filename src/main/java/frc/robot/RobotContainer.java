package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  // Sürücü kontrolcüsü
  public static final CommandXboxController driverController = new CommandXboxController(
      OIConstants.kDriverControllerPort);

  // Operatör kontrolcüsü
  public static final CommandXboxController operatorController = new CommandXboxController(
      1);

  // Alt sistemlerin tanımları
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();;

  // Otomatik komut seçici
  public static SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  // Yavaş hız modu bayrağı
  private boolean slowSpeedEnabled = false;

  // RobotContainer yapıcısı
  public RobotContainer() {
    // Varsayılan komutu ayarla
    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriveDeadband) / 1.4,
                true,
                slowSpeedEnabled),
            m_robotDrive));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    driverController.start().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));

    driverController.leftBumper().onTrue(new InstantCommand(() -> slowSpeedEnabled = !slowSpeedEnabled));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}