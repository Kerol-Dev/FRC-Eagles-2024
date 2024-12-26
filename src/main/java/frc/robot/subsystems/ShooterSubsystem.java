package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax shooterMotorUpper = new CANSparkMax(7, MotorType.kBrushless); // Üst atıcı motoru
    private final CANSparkMax shooterMotorLower = new CANSparkMax(8, MotorType.kBrushless); // Alt atıcı motoru
    public static final CANSparkMax shooterMotorHinge = new CANSparkMax(11, MotorType.kBrushless); // Atıcı menteşe motoru

    private SparkPIDController shooterMotorUpperPID; // Üst atıcı motoru PID kontrolcüsü
    private SparkPIDController shooterMotorLowerPID; // Alt atıcı motoru PID kontrolcüsü
    private SparkPIDController shooterMotorHingePID; // Atıcı menteşe motoru PID kontrolcüsü

    private double velocityKp = 0.0006; // Hız PID P parametresi
    private double velocityKi = 0; // Hız PID I parametresi
    private double velocityKd = 0; // Hız PID D parametresi

    private double positionKp = 0.25; // Pozisyon PID P parametresi
    private double positionKi = 0; // Pozisyon PID I parametresi
    private double positionKd = 0; // Pozisyon PID D parametresi
    private double positionMaxOutput = 0.6; // Pozisyon maksimum çıkış

    private int RPMTolerance = 250; // RPM toleransı
    private double angleTolerance = 0.20; // Açı toleransı

    public double goalAngle = 0; // Hedef açı

    public ShooterSubsystem() {
        shooterMotorUpper.restoreFactoryDefaults(); // Üst motoru fabrika ayarlarına sıfırlama
        shooterMotorUpper.setIdleMode(IdleMode.kBrake); // Üst motor boşta frenleme modu
        shooterMotorUpper.setInverted(false); // Üst motor yönü
        shooterMotorUpperPID = shooterMotorUpper.getPIDController(); // Üst motor PID kontrolcüsünü alma
        shooterMotorUpperPID.setFeedbackDevice(shooterMotorUpper.getEncoder()); // Geri bildirim cihazı olarak enkoder kullanma

        shooterMotorUpperPID.setP(velocityKp); // Hız PID P parametresini ayarlama
        shooterMotorUpperPID.setI(velocityKi); // Hız PID I parametresini ayarlama
        shooterMotorUpperPID.setD(velocityKd); // Hız PID D parametresini ayarlama

        shooterMotorLower.restoreFactoryDefaults(); // Alt motoru fabrika ayarlarına sıfırlama
        shooterMotorLower.setIdleMode(IdleMode.kBrake); // Alt motor boşta frenleme modu
        shooterMotorLower.setInverted(false); // Alt motor yönü
        shooterMotorLowerPID = shooterMotorLower.getPIDController(); // Alt motor PID kontrolcüsünü alma
        shooterMotorLowerPID.setFeedbackDevice(shooterMotorLower.getEncoder()); // Geri bildirim cihazı olarak enkoder kullanma

        shooterMotorLowerPID.setP(velocityKp); // Hız PID P parametresini ayarlama
        shooterMotorLowerPID.setI(velocityKi); // Hız PID I parametresini ayarlama
        shooterMotorLowerPID.setD(velocityKd); // Hız PID D parametresini ayarlama

        shooterMotorHinge.restoreFactoryDefaults(); // Menteşe motorunu fabrika ayarlarına sıfırlama
        shooterMotorHinge.setIdleMode(IdleMode.kCoast); // Menteşe motoru boşta sürüklenme modu
        shooterMotorHinge.setInverted(true); // Menteşe motoru yönü
        shooterMotorHingePID = shooterMotorHinge.getPIDController(); // Menteşe motoru PID kontrolcüsünü alma
        shooterMotorHingePID.setFeedbackDevice(shooterMotorHinge.getEncoder()); // Geri bildirim cihazı olarak enkoder kullanma

        shooterMotorHingePID.setP(positionKp); // Pozisyon PID P parametresini ayarlama
        shooterMotorHingePID.setI(positionKi); // Pozisyon PID I parametresini ayarlama
        shooterMotorHingePID.setD(positionKd); // Pozisyon PID D parametresini ayarlama
        shooterMotorHingePID.setOutputRange(-positionMaxOutput, positionMaxOutput); // Pozisyon maksimum çıkış aralığını ayarlama

        shooterMotorHinge.getEncoder().setPosition(0); // Enkoder pozisyonunu sıfırlama

        shooterMotorHinge.enableSoftLimit(SoftLimitDirection.kForward, true); // İleri yumuşak limiti etkinleştirme
        shooterMotorHinge.enableSoftLimit(SoftLimitDirection.kReverse, true); // Geri yumuşak limiti etkinleştirme

        shooterMotorHinge.setSoftLimit(SoftLimitDirection.kForward, 0); // İleri yumuşak limit değerini ayarlama
        shooterMotorHinge.setSoftLimit(SoftLimitDirection.kReverse, -141); // Geri yumuşak limit değerini ayarlama
        shooterMotorHingePID.setReference(-35, ControlType.kPosition); // Menteşe pozisyonunu ayarlama
    }

    @Override
    public void periodic() {
        if(goalAngle == 0)
        goalAngle = -4;

        SmartDashboard.putNumber("Goal Angle", goalAngle);
        SmartDashboard.putNumber("Shooter RPM", shooterMotorLower.getEncoder().getVelocity());

        SmartDashboard.putNumber("Shooter Position", shooterMotorHinge.getEncoder().getPosition()); // Menteşe pozisyonunu SmartDashboard'a yazdırma
    }

    public double getShooterRPM(boolean upper) {
        return upper ? shooterMotorUpper.getEncoder().getVelocity() : shooterMotorLower.getEncoder().getVelocity(); // Motor RPM değerini alma
    }

    public boolean shooterAtGoalRPM(int goalRPM) {
        return getShooterRPM(true) >= goalRPM || Math.abs(getShooterRPM(true) - goalRPM) <= RPMTolerance; // Motor hedef RPM değerinde mi
    }

    public boolean shooterHingeAtGoal() {
        return Math.abs(shooterMotorHinge.getEncoder().getPosition() - goalAngle) <= angleTolerance; // Menteşe hedef pozisyonda mı
    }

    public Command setShooterRPM(double goalRPM) {
        return Commands.runOnce(() -> setShooterRPMLocal(goalRPM)); // Motor RPM değerini ayarlama komutu
    }

    public void setShooterRPMLocal(double goalRPM) {
        if (goalRPM == 0) {
            stopShooterMotorsLocal(); // Motorları durdur
            return;
        }

        double percent = goalRPM / 5500.0; // RPM'yi yüzdelik değere çevir
        shooterMotorUpper.set(percent); // Üst motor hızını ayarla
        shooterMotorLower.set(percent); // Alt motor hızını ayarla
    }

    public Command setShooterAngle() {
        return Commands.run(() -> setShooterAngleLocal()).beforeStarting(Commands.runOnce(() -> goalAngle = LimelightHelpers.calculateShootingAngle())); // Atıcı açısını ayarlama komutu
    }

    public void setAmpAngle(int angle) {
        goalAngle = angle; // Atıcı açısını ayarlama komutu
        setShooterAngleLocal();
    }

    public Command stopShooterHinge() {
        return Commands.run(() -> shooterMotorHinge.set(0)); // Menteşe motorunu durdurma komutu
    }

    public Command resetShooterPosition() {
        return Commands.run(() -> shooterMotorHinge.getEncoder().setPosition(0)); // Menteşe pozisyonunu sıfırlama komutu
    }

    public void setShooterAngleLocal() {
        if (goalAngle <= 0)
            shooterMotorHingePID.setReference(goalAngle, ControlType.kPosition); // Menteşe pozisyonunu ayarla
    }

    public Command stopShooterMotors() {
        return Commands.runOnce(() -> stopShooterMotorsLocal()); // Motorları durdurma komutu
    }

    public void stopShooterMotorsLocal() {
        shooterMotorUpper.stopMotor(); // Üst motoru durdur
        shooterMotorLower.stopMotor(); // Alt motoru durdur
    }
}