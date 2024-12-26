package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax intakeMotor = new CANSparkMax(5, MotorType.kBrushless); // İç alım motoru
    private final CANSparkMax intakeMotor2 = new CANSparkMax(6, MotorType.kBrushless); // İkinci iç alım motoru
    private DigitalInput feederSensor = new DigitalInput(5); // Besleyici sensörü

    public IntakeSubsystem() {
        intakeMotor.restoreFactoryDefaults(); // Motoru fabrika ayarlarına sıfırlama
        intakeMotor.setIdleMode(IdleMode.kBrake); // Motor boşta frenleme modu
        intakeMotor.setInverted(false); // Motor yönü

        intakeMotor2.restoreFactoryDefaults(); // İkinci motoru fabrika ayarlarına sıfırlama
        intakeMotor2.setIdleMode(IdleMode.kBrake); // İkinci motor boşta frenleme modu
        intakeMotor2.setInverted(false); // İkinci motor yönü
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Has Note", !feederSensor.get()); // Feeder sensörü durumunu SmartDashboard'a yazdırma
    }

    public boolean hasNote() {
        return !feederSensor.get(); // Feeder sensör durumu
    }

    public Command setIntakeSpeed(double speed) {
        return Commands.runOnce(() -> setSpeeds(speed)); // İç alım hızını ayarlama komutu
    }

    public void setSpeeds(double speed) {
        intakeMotor.set(speed); // İç alım motoru hızını ayarlama
        intakeMotor2.set(speed); // İkinci iç alım motoru hızını ayarlama
    }

    public Command stopIntake() {
        return Commands.runOnce(() -> setSpeeds(0)); // İç alımı durdurma komutu
    }
}