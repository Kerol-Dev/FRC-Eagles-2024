package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.RotateToTarget;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  // Sürücü kontrolcüsü
  public static final CommandXboxController driverController = new CommandXboxController(
      OIConstants.kDriverControllerPort);

  // Operatör kontrolcüsü
  public static final CommandXboxController operatorController = new CommandXboxController(
      1);

  // Alt sistemlerin tanımları
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();
  public final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  public final FeederSubsystem m_FeederSubsystem = new FeederSubsystem();
  public final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();

  // Otomatik komut seçici
  public static SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  // Yavaş hız modu bayrağı
  private boolean slowSpeedEnabled = false;

  // RobotContainer yapıcısı
  public RobotContainer() {
    NamedCommands.registerCommand("ShootNote", automaticShootNote());
    NamedCommands.registerCommand("IntakeInit", intakeGrabNote());
    // Varsayılan komutu ayarla
    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriveDeadband) / 1.4,
                true,
                slowSpeedEnabled),
            m_robotDrive));

    // Düğme bağlamalarını yapılandır
    configureButtonBindings();
    // PathPlanner'ı yapılandır
    configurePathPlanner();
  }

  // PathPlanner'ı yapılandırma fonksiyonu
  private void configurePathPlanner() {
    // NamedCommands kullanarak komutları kayıt et
    NamedCommands.registerCommand("IntakeInit", intakeGrabNote());
    NamedCommands.registerCommand("ShootNote", new SequentialCommandGroup(automaticShootNote(), new WaitCommand(0.25), m_ShooterSubsystem.stopShooterMotors()));
    // Otomatik komut seçici yapılandır
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(autoChooser);
  }

  // Düğme bağlamalarını yapılandırma fonksiyonu
  private void configureButtonBindings() {
    // Başlığı sıfırlama
    driverController.start().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));

    // Nota tutma komutu
    driverController.b().whileTrue(
        intakeGrabNote()).onFalse(stopIntake().alongWith(resetRumble()).alongWith(m_FeederSubsystem.stopFeeder()));

    // Açıyı ayarla ve atış yap
    driverController.rightTrigger().whileTrue(automaticShootNote())
        .onFalse(resetShooter().alongWith(m_FeederSubsystem.stopFeeder()));

    // Sağ tampon düğmesi ile ampul atışı yap
    driverController.rightBumper().whileTrue(shootAmp()).onFalse(resetShooter().alongWith(m_FeederSubsystem.stopFeeder()));

    // Hedefe dönme komutu
    driverController.leftTrigger().whileTrue(new RotateToTarget(m_robotDrive))
        .onFalse(new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, true, false)));

    // Tırmanıcıyı etkinleştir
    operatorController.povUp().onTrue(enableClimber());

    // Tırmanıcıyı devre dışı bırak
    operatorController.povDown().onFalse(disableClimber());

    // Açıyı manuel ayarla (Test için)
    operatorController.start().onTrue(m_ClimbSubsystem.goToZeroPosition());

    // Tırmanıcı hızı ayarlamaları
    operatorController.leftBumper().whileTrue(m_ClimbSubsystem.setClimbSpeed(0.2))
        .onFalse(m_ClimbSubsystem.stopClimber());
    operatorController.leftTrigger().whileTrue(m_ClimbSubsystem.setClimbSpeed(-0.2))
        .onFalse(m_ClimbSubsystem.stopClimber());

    operatorController.rightBumper().whileTrue(m_ClimbSubsystem.setClimbSpeed(0.8))
        .onFalse(m_ClimbSubsystem.stopClimber());
    operatorController.rightTrigger().whileTrue(m_ClimbSubsystem.setClimbSpeed(-0.8))
        .onFalse(m_ClimbSubsystem.stopClimber());

    // Otomatik fırlatma komutu
    driverController.a().whileTrue(ejectShoot())
        .onFalse(resetShooter());

    // Yavaş hız geçişi
    driverController.leftBumper().onTrue(new InstantCommand(() -> slowSpeedEnabled = !slowSpeedEnabled));
  }

  // Komutlar
  // Nota tutma komutu
  private Command intakeGrabNote() {
    return new ParallelDeadlineGroup(new WaitUntilCommand(() -> m_IntakeSubsystem.hasNote()),
      m_IntakeSubsystem.setIntakeSpeed(0.45).alongWith(m_FeederSubsystem.setFeederSpeed(0.35)))
        .andThen(stopIntake().alongWith(m_FeederSubsystem.stopFeeder()))
        .andThen(new InstantCommand(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0.5)))
        .andThen(new WaitCommand(0.5))
        .andThen(resetRumble());
  }

  // Atıcıyı sıfırlama komutu
  private Command resetShooter() {
    return stopShooter().alongWith(stopIntake()).alongWith(resetShooterAngle());
  }

  // Alma sistemini durdurma komutu
  private Command stopIntake() {
    return m_IntakeSubsystem.stopIntake();
  }

  private Command resetShooterAngle() {
    return Commands.runOnce(() -> m_ShooterSubsystem.goalAngle = 0);
  }

  // Titreşimi sıfırlama komutu
  private Command resetRumble() {
    return new InstantCommand(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0))
        .andThen(new InstantCommand(() -> operatorController.getHID().setRumble(RumbleType.kBothRumble, 0)));
  }

  // Fırlatma komutu
  private Command ejectShoot() {
    return new ParallelDeadlineGroup(checkPossibilityNoVision(),
        m_ShooterSubsystem.setShooterRPM(2000).alongWith(m_ShooterSubsystem.setShooterAngle())
            .alongWith(new WaitUntilCommand(
                () -> m_ShooterSubsystem.shooterAtGoalRPM(2000) && m_ShooterSubsystem.shooterHingeAtGoal())
                .andThen(m_FeederSubsystem.setFeederSpeed(1))));
  }

  // Ampul atışı komutu
  private Command shootAmp() {
    return new SequentialCommandGroup(m_ShooterSubsystem.setAmpAngle(-95), new WaitUntilCommand(() -> m_ShooterSubsystem.shooterHingeAtGoal()), m_ShooterSubsystem.setShooterRPM(1100), new WaitUntilCommand(() -> m_ShooterSubsystem.shooterAtGoalRPM(1100)), m_FeederSubsystem.setFeederSpeed(1));
  }

  // Otomatik nota atışı komutu
  private Command automaticShootNote() {
    return new ParallelDeadlineGroup(checkPossibility(), new RotateToTarget(m_robotDrive),
        m_ShooterSubsystem.setShooterRPM(5500).alongWith(m_ShooterSubsystem.setShooterAngle())
            .alongWith(new WaitUntilCommand(
                () -> m_ShooterSubsystem.shooterAtGoalRPM(5500) && m_ShooterSubsystem.shooterHingeAtGoal())
                .andThen(new WaitCommand(0.3)).andThen(m_FeederSubsystem.setFeederSpeed(1))));
  }

  // Olasılığı kontrol etme komutu
  private Command checkPossibility() {
    return new WaitUntilCommand(() -> !m_IntakeSubsystem.hasNote())
        .andThen(new InstantCommand(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0.5)));
  }

  // Görüş olmadan olasılığı kontrol etme komutu
  private Command checkPossibilityNoVision() {
    return new WaitUntilCommand(() -> !m_IntakeSubsystem.hasNote())
        .andThen(new InstantCommand(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0.5)));
  }

  // Atıcıyı durdurma komutu
  private Command stopShooter() {
    return m_ShooterSubsystem.stopShooterMotors()
        .alongWith(resetRumble());
  }

  // Tırmanıcıyı etkinleştirme komutu
  private Command enableClimber() {
    return m_ClimbSubsystem.setClimbPosition(true);
  }

  // Tırmanıcıyı devre dışı bırakma komutu
  private Command disableClimber() {
    return m_ClimbSubsystem.setClimbPosition(false);
  }

  // Otonom komutu getirme fonksiyonu
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}