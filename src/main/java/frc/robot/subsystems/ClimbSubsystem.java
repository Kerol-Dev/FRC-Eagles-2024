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

public class ClimbSubsystem extends SubsystemBase {
    public final CANSparkMax climbMotor = new CANSparkMax(23, MotorType.kBrushless); // Tırmanma motoru
    private SparkPIDController climbMotorPID; // PID kontrolcüsü

    private double positionKp = 5; // PID P parametresi
    private double positionKi = 0; // PID I parametresi
    private double positionKd = 0; // PID D parametresi

    public ClimbSubsystem() {
        climbMotor.restoreFactoryDefaults(); // Motor fabrika ayarlarını geri yükleme
        climbMotor.setIdleMode(IdleMode.kBrake); // Motor boşta frenleme modu
        climbMotor.setInverted(false); // Motor yönünü ayarlama
        climbMotorPID = climbMotor.getPIDController(); // PID kontrolcüsünü alma
        climbMotorPID.setFeedbackDevice(climbMotor.getEncoder()); // Geri bildirim cihazı olarak enkoder kullanma

        climbMotorPID.setP(positionKp); // PID P parametresini ayarlama
        climbMotorPID.setI(positionKi); // PID I parametresini ayarlama
        climbMotorPID.setD(positionKd); // PID D parametresini ayarlama

        climbMotorPID.setOutputRange(-0.8, 0.8); // PID çıkış aralığını ayarlama

        climbMotor.enableSoftLimit(SoftLimitDirection.kForward, true); // İleri yumuşak limiti etkinleştirme
        climbMotor.enableSoftLimit(SoftLimitDirection.kReverse, true); // Geri yumuşak limiti etkinleştirme

        climbMotor.setSoftLimit(SoftLimitDirection.kForward, 30); // İleri yumuşak limit değerini ayarlama
        climbMotor.setSoftLimit(SoftLimitDirection.kReverse, -65); // Geri yumuşak limit değerini ayarlama
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb Position", climbMotor.getEncoder().getPosition()); // Tırmanma pozisyonunu SmartDashboard'a yazdırma
    }

    public Command setClimbPosition(boolean up) {
        return Commands.runOnce(() -> climbMotorPID.setReference(up ? -65 : 29, ControlType.kPosition)); // Tırmanma pozisyonunu ayarlama komutu
    }

    public Command goToZeroPosition() {
        return Commands.runOnce(() -> climbMotorPID.setReference(0, ControlType.kPosition)); // Sıfır pozisyonuna gitme komutu
    }

    public Command resetClimbPosition() {
        return Commands.runOnce(() -> climbMotor.getEncoder().setPosition(0)); // Tırmanma pozisyonunu sıfırlama komutu
    }

    public Command setClimbSpeed(double speed) {
        return Commands.runOnce(() -> climbMotor.set(-speed)); // Tırmanma hızını ayarlama komutu
    }

    public Command stopClimber() {
        return Commands.runOnce(() -> climbMotor.stopMotor()); // Tırmanma motorunu durdurma komutu
    }
}