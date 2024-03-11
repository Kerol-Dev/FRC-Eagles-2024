package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;

public class MoveToAprilTag extends Command {
  private DriveSubsystem drivetrain;
  private PIDController pidRotation;
  private PIDController pidForward;
  private PIDController pidSideways;

  /** Creates a new AlignToTarget. */
  public MoveToAprilTag(DriveSubsystem dt) {
    drivetrain = dt;

    pidForward = new PIDController(0.4, 0, 0);
    pidForward.setTolerance(0.05);

    pidSideways = new PIDController(0.4, 0, 0);
    pidSideways.setTolerance(0.05);

    pidRotation = new PIDController(0.02, 0, 0);
    pidRotation.setTolerance(2);

    addRequirements(dt);
  }

  @Override
  public void execute() {
    double rotation = pidRotation.calculate(LimelightHelpers.getTX(""), 0);
    drivetrain.drive(0, 0, -rotation, false, false);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, false, false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}