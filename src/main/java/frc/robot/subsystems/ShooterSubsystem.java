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
import frc.robot.LimelightHelpers;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax shooterMotorUpper = new CANSparkMax(31, MotorType.kBrushless);
    private final CANSparkMax shooterMotorLower = new CANSparkMax(41, MotorType.kBrushless);
    private final CANSparkMax shooterMotorHinge = new CANSparkMax(51, MotorType.kBrushless);

    private SparkPIDController shooterMotorUpperPID;
    private SparkPIDController shooterMotorLowerPID;
    private SparkPIDController shooterMotorHingePID;

    private double velocityKp = 1;
    private double velocityKi = 0;
    private double velocityKd = 0;

    private double positionKp = 1;
    private double positionKi = 0;
    private double positionKd = 0;
    private double positionMaxOutput = 0.4;

    private int RPMTolerance = 50;
    private int angleTolerance = 2;

    public double goalAngle = 0;
    public double ovverideAngle = 0;

    public ShooterSubsystem() {
        shooterMotorUpper.restoreFactoryDefaults();
        shooterMotorUpper.setIdleMode(IdleMode.kBrake);
        shooterMotorUpper.setInverted(false);
        shooterMotorUpperPID.setFeedbackDevice(shooterMotorUpper.getEncoder());

        shooterMotorUpperPID.setP(velocityKp);
        shooterMotorUpperPID.setI(velocityKi);
        shooterMotorUpperPID.setD(velocityKd);

        shooterMotorLower.restoreFactoryDefaults();
        shooterMotorLower.setIdleMode(IdleMode.kBrake);
        shooterMotorLower.setInverted(false);
        shooterMotorLowerPID.setFeedbackDevice(shooterMotorLower.getEncoder());

        shooterMotorLowerPID.setP(velocityKp);
        shooterMotorLowerPID.setI(velocityKi);
        shooterMotorLowerPID.setD(velocityKd);

        shooterMotorHinge.restoreFactoryDefaults();
        shooterMotorHinge.setIdleMode(IdleMode.kBrake);
        shooterMotorHinge.setInverted(false);
        shooterMotorHingePID.setFeedbackDevice(shooterMotorHinge.getEncoder());

        shooterMotorHingePID.setP(positionKp);
        shooterMotorHingePID.setI(positionKi);
        shooterMotorHingePID.setD(positionKd);
        shooterMotorHingePID.setOutputRange(-positionMaxOutput, positionMaxOutput);

        shooterMotorHinge.enableSoftLimit(SoftLimitDirection.kForward, true);
        shooterMotorHinge.enableSoftLimit(SoftLimitDirection.kReverse, true);

        shooterMotorHinge.setSoftLimit(SoftLimitDirection.kForward, 90);
        shooterMotorHinge.setSoftLimit(SoftLimitDirection.kReverse, 0);
    }

    @Override
    public void periodic() {
        goalAngle = LimelightHelpers.calculateShootingAngle();

        SmartDashboard.putNumber("Upper Shooter RPM", shooterMotorUpper.getEncoder().getVelocity());
        SmartDashboard.putNumber("Lower Shooter RPM", shooterMotorLower.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter Position", shooterMotorHinge.getEncoder().getPosition());
    }

    public double getShooterRPM(boolean upper) {
        return upper ? shooterMotorUpper.getEncoder().getVelocity() : shooterMotorLower.getEncoder().getVelocity();
    }

    public boolean shooterAtGoalRPM(int goalRPM) {
        return Math.abs(getShooterRPM(true) - goalRPM) <= RPMTolerance
                && Math.abs(getShooterRPM(false) - goalRPM) <= RPMTolerance;
    }

    public boolean shooterHingeAtGoal() {
        return Math.abs(shooterMotorHinge.getEncoder().getPosition() - goalAngle) <= angleTolerance;
    }

    public Command setShooterRPM(int goalRPM) {
        return Commands.runOnce(() -> setShooterRPMLocal(goalRPM));
    }

    private void setShooterRPMLocal(int goalRPM) {
        if (goalRPM == 0) {
            stopShooterMotorsLocal();
            return;
        }

        shooterMotorUpperPID.setReference(goalRPM, ControlType.kVelocity);
        shooterMotorLowerPID.setReference(goalRPM, ControlType.kVelocity);
    }

    public Command setShooterAngle() {
        return Commands.runOnce(() -> setShooterAngleLocal());
    }

    public Command setOvverideAngle(double angle)
    {
        return Commands.runOnce(() -> ovverideAngle = angle);
    }

    public void setShooterAngleLocal() {
        if (goalAngle >= 0 && ovverideAngle <= 0)
            shooterMotorHingePID.setReference(goalAngle, ControlType.kPosition);

        if (ovverideAngle > 0)
            shooterMotorHingePID.setReference(ovverideAngle, ControlType.kPosition);
    }

    public Command stopShooterMotors() {
        return Commands.runOnce(() -> stopShooterMotorsLocal());
    }

    public void stopShooterMotorsLocal() {
        shooterMotorUpperPID.setReference(2500, ControlType.kVelocity);
        shooterMotorLowerPID.setReference(2500, ControlType.kVelocity);
    }
}