package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class RotateToTarget extends Command {
  private DriveSubsystem drivetrain;
  private PIDController pidRotation;
  private boolean isFinished = false;

  public RotateToTarget(DriveSubsystem dt) {
    drivetrain = dt;

    pidRotation = new PIDController(0.01, 0, 0);
    pidRotation.setTolerance(1);

    addRequirements(dt);
  }

  @Override
  public void execute() {
    if(!LimelightHelpers.getTV(""))
    {
      RobotContainer.driverController.getHID().setRumble(RumbleType.kBothRumble, 0.5);
    }

    double rotation = pidRotation.calculate(LimelightHelpers.getTX(""), 0);
    drivetrain.drive(0, 0, rotation, false, false);

    if(pidRotation.atSetpoint())
    {
      isFinished = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    isFinished = false;
    drivetrain.drive(0, 0, 0, false, false);
    RobotContainer.driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}