package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.RotateToTarget;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
  public final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  public final FeederSubsystem m_FeederSubsystem = new FeederSubsystem();
  public final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  public final VisionSubsystem m_VisionSubsystem = new VisionSubsystem(m_robotDrive);

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
                -MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriveDeadband) / 1.4,
                true,
                slowSpeedEnabled),
            m_robotDrive));

    m_ShooterSubsystem
        .setDefaultCommand(new RunCommand(() -> m_ShooterSubsystem.setShooterAngleLocal(), m_ShooterSubsystem));

    // Düğme bağlamalarını yapılandır
    configureButtonBindings();
    // PathPlanner'ı yapılandır
    configurePathPlanner();
  }

  // PathPlanner'ı yapılandırma fonksiyonux"
  private void configurePathPlanner() {
    // NamedCommands kullanarak komutları kayıt et
    NamedCommands.registerCommand("IntakeInit", intakeGrabNote());
    NamedCommands.registerCommand("InitShooter", Commands.runOnce(() -> m_ShooterSubsystem.setShooterRPMLocal(5500), m_ShooterSubsystem));
    NamedCommands.registerCommand("ShootNote", new ConditionalCommand(new RotateToTarget(m_robotDrive).withTimeout(2).andThen(automaticShootNote()).andThen(Commands.waitSeconds(0.5)).andThen(resetShooterAuto()),
        new WaitCommand(0.7), m_IntakeSubsystem::hasNote));
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
        resetShooter().andThen(intakeGrabNote()))
        .onFalse(stopIntake().alongWith(resetRumble()).alongWith(m_FeederSubsystem.stopFeeder()));

    // Açıyı ayarla ve atış yap
    driverController.rightTrigger().whileTrue(automaticShootNote())
        .onFalse(resetShooter().alongWith(m_FeederSubsystem.stopFeeder()));

    // Sağ tampon düğmesi ile ampul atışı yap
    driverController.rightBumper().whileTrue(shootAmp())
        .onFalse(resetShooter().alongWith(m_FeederSubsystem.stopFeeder()).alongWith(m_IntakeSubsystem.stopIntake()));

    // Hedefe dönme komutu
    driverController.leftTrigger().whileTrue(new RotateToTarget(m_robotDrive))
        .onFalse(new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, true, false)));

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
        .andThen(new InstantCommand(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 1)))
        .andThen(new WaitCommand(1))
        .andThen(resetRumble());
  }

  // Atıcıyı sıfırlama komutu
  private Command resetShooter() {
    return stopShooter().alongWith(stopIntake()).alongWith(resetShooterAngle());
  }

  private Command resetShooterAuto() {
    return stopShooter().alongWith(stopIntake()).alongWith(resetShooterAngleAuto());
  }

  // Alma sistemini durdurma komutu
  private Command stopIntake() {
    return m_IntakeSubsystem.stopIntake();
  }

  private Command resetShooterAngle() {
    return Commands.runOnce(() -> m_ShooterSubsystem.goalAngle = 0);
  }

    private Command resetShooterAngleAuto() {
    return Commands.runOnce(() -> m_ShooterSubsystem.goalAngle = -25).andThen(Commands.runOnce(() -> m_ShooterSubsystem.setShooterAngleLocal()));
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
    return Commands.runOnce(() -> m_ShooterSubsystem.setAmpAngle(-141), m_ShooterSubsystem)
        .andThen(Commands.waitUntil(() -> m_ShooterSubsystem.shooterHingeAtGoal()))
        .andThen(Commands.runOnce(() -> m_ShooterSubsystem.setAmpAngle(-134), m_ShooterSubsystem))
        .andThen(Commands.waitUntil(() -> m_ShooterSubsystem.shooterHingeAtGoal()))
        .andThen(Commands.runOnce(() -> m_ShooterSubsystem.setShooterRPMLocal(1400), m_ShooterSubsystem))
        .andThen(Commands.waitSeconds(0.8))
        .andThen(Commands.runOnce(() -> m_FeederSubsystem.setSpeeds(1), m_FeederSubsystem));
  }

  // Otomatik nota atışı komutu
  private Command automaticShootNote() {
    return new ParallelDeadlineGroup(checkPossibilityNoVision(),
        m_ShooterSubsystem.setShooterRPM(5500).alongWith(m_ShooterSubsystem.setShooterAngle())
            .alongWith(new WaitUntilCommand(
                () -> m_ShooterSubsystem.shooterAtGoalRPM(5000) && m_ShooterSubsystem.shooterHingeAtGoal())
                .andThen(new WaitCommand(0.3)).andThen(m_FeederSubsystem.setFeederSpeed(1))));
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

  // Otonom komutu getirme fonksiyonu
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}