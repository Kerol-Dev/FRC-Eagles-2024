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
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  public static final CommandXboxController driverController = new CommandXboxController(
      OIConstants.kDriverControllerPort);

      public static final CommandXboxController operatorController = new CommandXboxController(
        1);
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();
  public final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  public final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();

  public static SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  private boolean slowSpeedEnabled = false;

  public RobotContainer() {
    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriveDeadband),
                true,
                slowSpeedEnabled),
            m_robotDrive));

    // m_ShooterSubsystem.setDefaultCommand(new RunCommand(() ->
    // m_ShooterSubsystem.setShooterAngleLocal(), m_ShooterSubsystem));

    configureButtonBindings();
    configurePathPlanner();
  }

  private void configurePathPlanner() {
    NamedCommands.registerCommand("IntakeInit", intakeGrabNote());
    NamedCommands.registerCommand("ShootNote", automaticShootNote()
        .andThen(new WaitUntilCommand(() -> !m_IntakeSubsystem.hasNote()).andThen(new WaitCommand(0.3))));
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(autoChooser);
  }

  private void configureButtonBindings() {
    // Zero Heading
    driverController.start().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));

    // Intake Grab Note
    operatorController.x().whileTrue(
        intakeGrabNote()).onFalse(stopIntake().alongWith(resetRumble()));

    // Set Angle And Shoot
    driverController.rightTrigger().whileTrue(automaticShootNote())
        .onFalse(resetShooter());

    // Rotate To Target
    driverController.leftTrigger().whileTrue(new RotateToTarget(m_robotDrive))
        .onFalse(new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, true, false)));

    // Enable Climber
    operatorController.povUp().whileTrue(enableClimber()).onFalse(stopClimber());

    // Disable Climber
    operatorController.povDown().whileTrue(disableClimber()).onFalse(stopClimber());

    // Automatic AMP Shoot
    driverController.b().whileTrue(automaticAmpShoot())
        .onFalse(resetShooter());

    // Reset Shooter
    driverController.povRight().whileTrue(
        m_ShooterSubsystem.setShooterBack().alongWith(new WaitUntilCommand(() -> !m_ShooterSubsystem.shooterHome.get())
            .andThen(m_ShooterSubsystem.stopShooterHinge().alongWith(m_ShooterSubsystem.resetShooterPosition()))));

    // Set Angle Manual (Testing)
    driverController.y().whileTrue(m_ShooterSubsystem.setShooterAngle());

    // Automatic Eject Shoot
    operatorController.a().whileTrue(ejectShoot())
        .onFalse(resetShooter());

    // Toggle Slow Speed
    driverController.leftBumper().onTrue(new InstantCommand(() -> slowSpeedEnabled = !slowSpeedEnabled));
  }

  // Commands
  private Command intakeGrabNote() {
    return new ParallelDeadlineGroup(new WaitUntilCommand(() -> m_IntakeSubsystem.hasNote()),
        m_IntakeSubsystem.setIntakeSpeed(0.4))
        .andThen(stopIntake())
        .andThen(new InstantCommand(() -> operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.5)))
        .andThen(new WaitCommand(0.5))
        .andThen(resetRumble());
  }

  private Command resetShooter() {
    return stopShooter().alongWith(m_ShooterSubsystem.setOvverideAngle(0)).alongWith(stopIntake());
  }

  private Command stopIntake() {
    return m_IntakeSubsystem.stopIntake();
  }

  private Command resetRumble() {
    return new InstantCommand(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0)).andThen(new InstantCommand(() -> operatorController.getHID().setRumble(RumbleType.kBothRumble, 0)));
  }

  private Command automaticAmpShoot() {
    return new ParallelDeadlineGroup(checkPossibilityNoVision(),
        m_ShooterSubsystem.setShooterRPM(1)
        .alongWith(m_ShooterSubsystem.setOvverideAngle(-5))
        .alongWith(m_ShooterSubsystem.setShooterAngle())
            .alongWith(new WaitUntilCommand(
                () -> m_ShooterSubsystem.shooterAtGoalRPM(5400) && m_ShooterSubsystem.shooterHingeAtGoal())
                .andThen(m_IntakeSubsystem.setIntakeSpeed(1))));
  }

  private Command ejectShoot() {
    return new ParallelDeadlineGroup(checkPossibilityNoVision(),
        m_ShooterSubsystem.setShooterRPM(0.4).alongWith(m_ShooterSubsystem.setShooterAngle())
            .alongWith(new WaitUntilCommand(
                () -> m_ShooterSubsystem.shooterAtGoalRPM(2000) && m_ShooterSubsystem.shooterHingeAtGoal())
                .andThen(m_IntakeSubsystem.setIntakeSpeed(1))));
  }

  private Command automaticShootNote() {
    return new ParallelDeadlineGroup(checkPossibility(), new RotateToTarget(m_robotDrive),
        m_ShooterSubsystem.setShooterRPM(1).alongWith(m_ShooterSubsystem.setShooterAngle())
            .alongWith(new WaitUntilCommand(
                () -> m_ShooterSubsystem.shooterAtGoalRPM(5400) && m_ShooterSubsystem.shooterHingeAtGoal())
                .andThen(m_IntakeSubsystem.setIntakeSpeed(1))));
  }

  private Command checkPossibility() {
    return new WaitUntilCommand(() -> !LimelightHelpers.isPossible() ||
        !m_IntakeSubsystem.hasNote())
        .andThen(new InstantCommand(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0.5)));
  }

  private Command checkPossibilityNoVision() {
    return new WaitUntilCommand(() -> !LimelightHelpers.isPossible() || !m_IntakeSubsystem.hasNote())
        .andThen(new InstantCommand(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0.5)));
  }

  private Command stopShooter() {
    return m_ShooterSubsystem.stopShooterMotors()
        .alongWith(resetRumble());
  }

  private Command enableClimber() {
    return m_ClimbSubsystem.setClimbPosition(true);
  }

  private Command disableClimber() {
    return m_ClimbSubsystem.setClimbPosition(false);
  }

  private Command stopClimber() {
    return m_ClimbSubsystem.stopClimber();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}