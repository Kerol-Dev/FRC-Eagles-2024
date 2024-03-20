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
    public final CANSparkMax climbMotor = new CANSparkMax(13, MotorType.kBrushless);
    private SparkPIDController climbMotorPID;

    private double positionKp = 5;
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

        climbMotorPID.setOutputRange(-0.8, 0.8);

        climbMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        climbMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

        climbMotor.setSoftLimit(SoftLimitDirection.kForward, 30);
        climbMotor.setSoftLimit(SoftLimitDirection.kReverse, -55);

        climbMotorPID.setReference(0, ControlType.kPosition);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb Position", climbMotor.getEncoder().getPosition());
    }

    public Command setClimbPosition(boolean up)
    {
        return Commands.runOnce(() -> climbMotorPID.setReference(up ? -54 : 29, ControlType.kPosition));
    }

    public Command resetClimbPosition()
    {
        return Commands.runOnce(() -> climbMotor.getEncoder().setPosition(0));
    }

    public Command setClimbSpeed(double speed)
    {
        return Commands.runOnce(() -> climbMotor.set(-speed));
    }

    public Command stopClimber()
    {
        return Commands.runOnce(() -> climbMotor.stopMotor());
    }
}