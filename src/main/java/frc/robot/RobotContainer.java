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
import frc.utils.CustomCommandRunner;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  public final CommandXboxController driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();
  public final FeederSubsystem m_FeederSubsystem = new FeederSubsystem();
  public final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  public final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();

  public static SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  public RobotContainer() {
    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriveDeadband),
                true,
                driverController.b().getAsBoolean()),
            m_robotDrive));

    configureButtonBindings();
    configurePathPlanner();
  }

  private void configurePathPlanner() {
    NamedCommands.registerCommand("IntakeInit", intakeGrabNote());
    NamedCommands.registerCommand("ShootNote", automaticMovingShootNote());
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData(autoChooser);
  }

  private void configureButtonBindings() {
    // Zero Heading
    driverController.start().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));

    // Intake Grab Note
    driverController.x().toggleOnTrue(new CustomCommandRunner(intakeGrabNote(), stopIntake().alongWith(stopFeeder()).alongWith(resetRumble())));

    // Set Angle And Shoot
    driverController.rightTrigger().whileTrue(automaticShootNote()).onFalse(stopShooter().alongWith(stopFeeder()));

    // Rotate To Target
    driverController.leftTrigger().whileTrue(new RotateToTarget(m_robotDrive))
        .onFalse(new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, true, false)));

    // Enable Climber
    driverController.rightBumper().toggleOnTrue(new CustomCommandRunner(enableClimber(), stopClimber()));

    // Disable Climber
    driverController.leftBumper().toggleOnTrue(new CustomCommandRunner(disableClimber(), stopClimber()));

    // Adjust Shooter Angle
    driverController.y().whileTrue(m_ShooterSubsystem.setShooterAngle());
  }

  // Commands
  private Command intakeGrabNote() {
    return new SequentialCommandGroup(
        m_IntakeSubsystem.setIntakeSpeed(1).alongWith(m_FeederSubsystem.setFeederSpeed(0.4)),
        new WaitUntilCommand(() -> m_FeederSubsystem.hasNote()),
        m_IntakeSubsystem.stopIntake().alongWith(m_FeederSubsystem.stopFeeder()));
  }

  private Command stopIntake() {
    return m_IntakeSubsystem.stopIntake();
  }

  private Command stopFeeder() {
    return m_FeederSubsystem.stopFeeder();
  }

  private Command resetRumble() {
    return new InstantCommand(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0));
  }

  private Command automaticShootNote() {
    return new ParallelRaceGroup(
        new WaitUntilCommand(() -> !LimelightHelpers.isPossible() || !m_FeederSubsystem.hasNote())
            .andThen(new InstantCommand(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0.5))),
        new SequentialCommandGroup(new RotateToTarget(m_robotDrive),
            m_ShooterSubsystem.setShooterRPM(5500).alongWith(m_ShooterSubsystem.setShooterAngle()),
            new WaitUntilCommand(
                () -> m_ShooterSubsystem.shooterAtGoalRPM(5500) && m_ShooterSubsystem.shooterHingeAtGoal()),
            m_FeederSubsystem.setFeederSpeed(1)));
  }

  private Command automaticMovingShootNote() {
    return new ParallelRaceGroup(
        new WaitUntilCommand(() -> !LimelightHelpers.isPossible() || !m_FeederSubsystem.hasNote())
            .andThen(new InstantCommand(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0.5))),
        new SequentialCommandGroup(
            m_ShooterSubsystem.setShooterRPM(5500).alongWith(m_ShooterSubsystem.setShooterAngle()),
            new WaitUntilCommand(
                () -> m_ShooterSubsystem.shooterAtGoalRPM(5500) && m_ShooterSubsystem.shooterHingeAtGoal()),
            m_FeederSubsystem.setFeederSpeed(1)));
  }

  private Command stopShooter() {
    return m_ShooterSubsystem.stopShooterMotors()
        .alongWith(new InstantCommand(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0)));
  }

  private Command enableClimber() {
    return m_ClimbSubsystem.setIntakePosition(true);
  }

  private Command disableClimber() {
    return m_ClimbSubsystem.setIntakePosition(false);
  }

  private Command stopClimber() {
    return m_ClimbSubsystem.stopClimber();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}