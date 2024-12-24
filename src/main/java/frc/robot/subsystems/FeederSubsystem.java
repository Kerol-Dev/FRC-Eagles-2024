package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {
    private final CANSparkMax feederMotor = new CANSparkMax(13, MotorType.kBrushless); // Besleyici motoru
    private final CANSparkMax feederMotor2 = new CANSparkMax(9, MotorType.kBrushless); // İkinci besleyici motoru

    public FeederSubsystem() {
        feederMotor.restoreFactoryDefaults(); // Motoru fabrika ayarlarına sıfırlama
        feederMotor.setIdleMode(IdleMode.kBrake); // Motor boşta frenleme modu
        feederMotor.setInverted(false); // Motor yönü

        feederMotor2.restoreFactoryDefaults(); // İkinci motoru fabrika ayarlarına sıfırlama
        feederMotor2.setIdleMode(IdleMode.kBrake); // İkinci motor boşta frenleme modu
        feederMotor2.setInverted(true); // İkinci motor yönü
    }

    public Command setFeederSpeed(double speed) {
        return Commands.runOnce(() -> setSpeeds(speed)); // Besleyici hızını ayarlama komutu
    }

    public void setSpeeds(double speed) {
        feederMotor.set(-speed); // Besleyici motoru hızını ayarlama
        feederMotor2.set(speed); // İkinci besleyici motoru hızını ayarlama
    }

    public Command stopFeeder() {
        return Commands.runOnce(() -> setSpeeds(0)); // Besleyiciyi durdurma komutu
    }
}