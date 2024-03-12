package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax intakeMotor = new CANSparkMax(25, MotorType.kBrushless);

    public IntakeSubsystem()
    {
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setInverted(false);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Position", intakeMotor.getEncoder().getPosition());
    }

    public Command setIntakeSpeed(double speed)
    {
        return Commands.runOnce(() -> intakeMotor.set(speed));
    }

    public Command stopIntake()
    {
        return Commands.runOnce(() -> intakeMotor.stopMotor());
    }
}