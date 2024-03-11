package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
    private final CANSparkMax climbMotor = new CANSparkMax(0, MotorType.kBrushless);
    private final SparkPIDController climbMotorPID;

    private double positionKp = 1;
    private double positionKi = 0;
    private double positionKd = 0;

    public ClimbSubsystem()
    {
        climbMotor.restoreFactoryDefaults();
        climbMotor.setIdleMode(IdleMode.kBrake);
        climbMotor.setInverted(false);
        climbMotorPID = climbMotor.getPIDController();
        climbMotorPID.setFeedbackDevice(climbMotor.getEncoder());

        climbMotorPID.setP(positionKp);
        climbMotorPID.setI(positionKi);
        climbMotorPID.setD(positionKd);

        climbMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        climbMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

        climbMotor.setSoftLimit(SoftLimitDirection.kForward, 100);
        climbMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
    }

    public Command setIntakePosition(boolean up)
    {
        return Commands.runOnce(() -> climbMotorPID.setReference(up ? 100 : 0, ControlType.kPosition), this);
    }

    public Command stopClimber()
    {
        return Commands.runOnce(() -> climbMotor.stopMotor(), this);
    }
}