package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {
    private final CANSparkMax feederMotor = new CANSparkMax(9, MotorType.kBrushless);
    private final CANSparkMax feederMotor2 = new CANSparkMax(10, MotorType.kBrushless);

    public FeederSubsystem()
    {
        feederMotor.restoreFactoryDefaults();
        feederMotor.setIdleMode(IdleMode.kBrake);
        feederMotor.setInverted(true);

        feederMotor2.restoreFactoryDefaults();
        feederMotor2.setIdleMode(IdleMode.kBrake);
        feederMotor2.setInverted(false);
    }

    public Command setFeederSpeed(double speed)
    {
        return Commands.runOnce(() -> setSpeeds(speed));
    }

    private void setSpeeds(double speed)
    {
        feederMotor.set(speed);
        feederMotor2.set(speed);
    }

    public Command stopFeeder()
    {
        return Commands.runOnce(() -> setSpeeds(0));
    }
}