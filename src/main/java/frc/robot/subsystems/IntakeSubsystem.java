package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax intakeMotor = new CANSparkMax(0, MotorType.kBrushless);

    public IntakeSubsystem()
    {
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setInverted(false);
    }

    public Command setIntakeSpeed(double speed)
    {
        return Commands.runOnce(() -> intakeMotor.set(speed), this);
    }

    public Command stopIntake()
    {
        return Commands.runOnce(() -> intakeMotor.stopMotor(), this);
    }
}