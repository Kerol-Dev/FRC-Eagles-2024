package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {
    private final CANSparkMax feederMotor = new CANSparkMax(16, MotorType.kBrushless);
    private final DigitalInput feederSensor = new DigitalInput(0);

    public FeederSubsystem() {
        feederMotor.restoreFactoryDefaults();
        feederMotor.setIdleMode(IdleMode.kBrake);
        feederMotor.setInverted(false);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Feeder Position", feederMotor.getEncoder().getPosition());
        SmartDashboard.putBoolean("Feeder Sensor", hasNote());
    }

    public boolean hasNote() {
        return !feederSensor.get();
    }

    public Command setFeederSpeed(double speed) {
        return Commands.runOnce(() -> feederMotor.set(speed));
    }

    public Command stopFeeder() {
        return Commands.runOnce(() -> feederMotor.stopMotor());
    }
}