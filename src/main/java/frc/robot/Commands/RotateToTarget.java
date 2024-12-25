package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class RotateToTarget extends Command {
  private DriveSubsystem drivetrain; // Sürüş sistemi referansı
  private PIDController pidRotation; // PID kontrolcüsü referansı
  private boolean isFinished = false; // Komut bitme durumu

  public RotateToTarget(DriveSubsystem dt) {
    drivetrain = dt; // Sürüş sistemini başlatma

    pidRotation = new PIDController(0.015, 0, 0); // PID kontrolcüsünü başlatma
    pidRotation.setTolerance(2); // PID toleransını ayarlama

    addRequirements(dt); // Gereksinimleri ekleme
  }

  @Override
  public void execute() {
    if (!LimelightHelpers.getTV("")) {
      RobotContainer.driverController.getHID().setRumble(RumbleType.kBothRumble, 0.5); // Limelight TV sinyali yoksa kumandayı titreştir
    }
    double rotation = pidRotation.calculate(LimelightHelpers.getTX(""), 0); // PID kontrolcüsü ile dönüş hesaplama

    drivetrain.drive(0, 0, rotation, false, false); // Sürüş sistemini döndürme

    if (pidRotation.atSetpoint()) {
      isFinished = true; // PID set noktasına ulaştıysa komutu bitirme durumu
    }
  }

  @Override
  public void end(boolean interrupted) {
    isFinished = false; // Komut bitme durumunu sıfırlama
    drivetrain.drive(0, 0, 0, false, false); // Sürüş sistemini durdurma
    RobotContainer.driverController.getHID().setRumble(RumbleType.kBothRumble, 0); // Kumanda titreşimini durdurma
  }

  @Override
  public boolean isFinished() {
    return isFinished; // Komut bitme durumunu döndürme
  }
}