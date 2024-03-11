package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Commands.MoveToAprilTag;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.utils.CustomCommandRunner;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final CommandXboxController driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();
  private final FeederSubsystem m_FeederSubsystem = new FeederSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();

  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  public RobotContainer() {
    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(driverController.getLeftY(), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(driverController.getLeftX(), OIConstants.kDriveDeadband),
                MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriveDeadband),
                true,
                driverController.rightBumper().getAsBoolean()),
            m_robotDrive));

    configureButtonBindings();
    configurePathPlanner();
  }

  private void configurePathPlanner() {
    autoChooser = AutoBuilder.buildAutoChooser();
  }

  private void configureButtonBindings() {
    // Zero Heading
    driverController.start().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));

    // Intake Grab Note
    driverController.x().toggleOnTrue(new CustomCommandRunner(intakeGrabNote(), stopIntake().alongWith(stopFeeder())));

    // Set Angle And Shoot
    driverController.rightTrigger().whileTrue(automaticShootNote()).onFalse(stopShooter().alongWith(stopFeeder()));

    // Rotate To Target
    driverController.leftTrigger().whileTrue(new MoveToAprilTag(m_robotDrive)).onFalse(new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, true, false)));

    // Enable Climber
    driverController.rightBumper().toggleOnTrue(new CustomCommandRunner(enableClimber(), stopClimber()));

    // Disable Climber
    driverController.leftBumper().toggleOnTrue(new CustomCommandRunner(disableClimber(), stopClimber()));
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

  private Command automaticShootNote() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0.5))
            .onlyIf(() -> !LimelightHelpers.isPossible()),
        new SequentialCommandGroup(m_ShooterSubsystem.setShooterRPM(5500).alongWith(m_ShooterSubsystem.setShooterAngle()),
            new WaitUntilCommand(
                () -> m_ShooterSubsystem.shooterAtGoalRPM(5500) && m_ShooterSubsystem.shooterHingeAtGoal()),
            m_FeederSubsystem.setFeederSpeed(1)));
  }

  private Command stopShooter() {
    return m_ShooterSubsystem.stopShooterMotors().alongWith( new InstantCommand(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0)));
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