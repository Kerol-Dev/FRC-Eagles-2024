package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {
    private final CANSparkMax feederMotor = new CANSparkMax(0, MotorType.kBrushless);
    private final DigitalInput feederSensor = new DigitalInput(0);

    public FeederSubsystem() {
        feederMotor.restoreFactoryDefaults();
        feederMotor.setIdleMode(IdleMode.kBrake);
        feederMotor.setInverted(false);
    }

    public boolean hasNote() {
        return !feederSensor.get();
    }

    public Command setFeederSpeed(double speed) {
        return Commands.runOnce(() -> feederMotor.set(speed), this);
    }

    public Command stopFeeder() {
        return Commands.runOnce(() -> feederMotor.stopMotor(), this);
    }
}