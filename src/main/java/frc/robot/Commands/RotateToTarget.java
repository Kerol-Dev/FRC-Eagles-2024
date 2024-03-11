package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;

public class RotateToTarget extends Command {
  private DriveSubsystem drivetrain;
  private PIDController pidRotation;
  private boolean isFinished = false;

  public RotateToTarget(DriveSubsystem dt) {
    drivetrain = dt;

    pidRotation = new PIDController(0.02, 0, 0);
    pidRotation.setTolerance(2);

    addRequirements(dt);
  }

  @Override
  public void execute() {
    double rotation = pidRotation.calculate(LimelightHelpers.getTX(""), 0);
    drivetrain.drive(0, 0, -rotation, false, false);

    if(pidRotation.atSetpoint())
    {
      isFinished = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    isFinished = false;
    drivetrain.drive(0, 0, 0, false, false);
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}