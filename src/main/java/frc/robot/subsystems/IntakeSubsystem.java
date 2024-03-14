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
    private final CANSparkMax intakeMotor = new CANSparkMax(9, MotorType.kBrushless);
    private final CANSparkMax intakeMotor2 = new CANSparkMax(10, MotorType.kBrushless);
    private DigitalInput feederSensor = new DigitalInput(0);

    public IntakeSubsystem()
    {
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setInverted(true);

        intakeMotor2.restoreFactoryDefaults();
        intakeMotor2.setIdleMode(IdleMode.kBrake);
        intakeMotor2.setInverted(false);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Feeder Sensor", !feederSensor.get());
    }

    public boolean hasNote()
    {
        return !feederSensor.get();
    }

    public Command setIntakeSpeed(double speed)
    {
        return Commands.runOnce(() -> setSpeeds(speed));
    }

    private void setSpeeds(double speed)
    {
        intakeMotor.set(speed);
        intakeMotor2.set(speed);
    }

    public Command stopIntake()
    {
        return Commands.runOnce(() -> setSpeeds(0));
    }
}