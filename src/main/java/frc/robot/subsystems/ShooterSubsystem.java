package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax shooterMotorUpper = new CANSparkMax(12, MotorType.kBrushless);
    private final CANSparkMax shooterMotorLower = new CANSparkMax(11, MotorType.kBrushless);
    private final CANSparkMax shooterMotorHinge = new CANSparkMax(14, MotorType.kBrushless);

    public final DigitalInput shooterHome = new DigitalInput(1);
    private SparkPIDController shooterMotorUpperPID;
    private SparkPIDController shooterMotorLowerPID;
    private SparkPIDController shooterMotorHingePID;

    private double velocityKp = 0.01;
    private double velocityKi = 0;
    private double velocityKd = 0;

    private double positionKp = 0.35;
    private double positionKi = 0;
    private double positionKd = 0;
    private double positionMaxOutput = 0.5;

    private int RPMTolerance = 200;
    private double angleTolerance = 2;

    public double goalAngle = 0;
    public double ovverideAngle = 0;

    public ShooterSubsystem() {
        shooterMotorUpper.restoreFactoryDefaults();
        shooterMotorUpper.setIdleMode(IdleMode.kCoast);
        shooterMotorUpper.setInverted(false);
        shooterMotorUpperPID = shooterMotorUpper.getPIDController();
        shooterMotorUpperPID.setFeedbackDevice(shooterMotorUpper.getEncoder());

        shooterMotorUpperPID.setP(velocityKp);
        shooterMotorUpperPID.setI(velocityKi);
        shooterMotorUpperPID.setD(velocityKd);

        shooterMotorLower.restoreFactoryDefaults();
        shooterMotorLower.setIdleMode(IdleMode.kCoast);
        shooterMotorLower.setInverted(false);
        shooterMotorLowerPID = shooterMotorLower.getPIDController();
        shooterMotorLowerPID.setFeedbackDevice(shooterMotorLower.getEncoder());

        shooterMotorLowerPID.setP(velocityKp);
        shooterMotorLowerPID.setI(velocityKi);
        shooterMotorLowerPID.setD(velocityKd);

        shooterMotorHinge.restoreFactoryDefaults();
        shooterMotorHinge.setIdleMode(IdleMode.kCoast);
        shooterMotorHinge.setInverted(false);
        shooterMotorHingePID = shooterMotorHinge.getPIDController();
        shooterMotorHingePID.setFeedbackDevice(shooterMotorHinge.getEncoder());

        shooterMotorHingePID.setP(positionKp);
        shooterMotorHingePID.setI(positionKi);
        shooterMotorHingePID.setD(positionKd);
        shooterMotorHingePID.setOutputRange(-positionMaxOutput, positionMaxOutput);

        shooterMotorHinge.getEncoder().setPosition(0);

        shooterMotorHinge.enableSoftLimit(SoftLimitDirection.kForward, true);
        shooterMotorHinge.enableSoftLimit(SoftLimitDirection.kReverse, true);

        shooterMotorHinge.setSoftLimit(SoftLimitDirection.kForward, -2);
        shooterMotorHinge.setSoftLimit(SoftLimitDirection.kReverse, -30);
        shooterMotorHingePID.setReference(-3, ControlType.kPosition);
    }

    @Override
    public void periodic() {
        goalAngle = ovverideAngle < 0 ? ovverideAngle : LimelightHelpers.calculateShootingAngle();

        SmartDashboard.putNumber("Upper Shooter RPM", shooterMotorUpper.getEncoder().getVelocity());
        SmartDashboard.putNumber("Lower Shooter RPM", shooterMotorLower.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter Position", shooterMotorHinge.getEncoder().getPosition());
        SmartDashboard.putBoolean("Home Shooter", !shooterHome.get());
    }

    public double getShooterRPM(boolean upper) {
        return upper ? shooterMotorUpper.getEncoder().getVelocity() : shooterMotorLower.getEncoder().getVelocity();
    }

    public boolean shooterAtGoalRPM(int goalRPM) {
        return getShooterRPM(true) >= goalRPM || Math.abs(getShooterRPM(true) - goalRPM) <= RPMTolerance;
    }

    public boolean shooterHingeAtGoal() {
        return Math.abs(shooterMotorHinge.getEncoder().getPosition() - goalAngle) <= angleTolerance;
    }

    public Command setShooterRPM(double goalRPM) {
        return Commands.runOnce(() -> setShooterRPMLocal(goalRPM));
    }

    private void setShooterRPMLocal(double goalRPM) {
        if (goalRPM == 0) {
            stopShooterMotorsLocal();
            return;
        }

        shooterMotorUpper.set(goalRPM);
        shooterMotorLower.set(goalRPM);
    }

    public Command setShooterAngle() {
        return Commands.run(() -> setShooterAngleLocal());
    }

    public Command setShooterBack() {
        return Commands.run(() -> shooterMotorHinge.set(0.15))
                .andThen(Commands.runOnce(() -> shooterMotorHinge.enableSoftLimit(SoftLimitDirection.kForward, false)));
    }

    public Command stopShooterHinge() {
        return Commands.run(() -> shooterMotorHinge.set(0))
                .andThen(Commands.runOnce(() -> shooterMotorHinge.enableSoftLimit(SoftLimitDirection.kForward, true)));
    }

    public Command resetShooterPosition() {
        return Commands.run(() -> shooterMotorHinge.getEncoder().setPosition(0));
    }

    public Command setOvverideAngle(double angle) {
        return Commands.runOnce(() -> ovverideAngle = angle);
    }

    public void setShooterAngleLocal() {
        if (goalAngle <= 0)
            shooterMotorHingePID.setReference(goalAngle, ControlType.kPosition);
    }

    public Command stopShooterMotors() {
        return Commands.runOnce(() -> stopShooterMotorsLocal());
    }

    public void stopShooterMotorsLocal() {
        shooterMotorUpper.stopMotor();
        shooterMotorLower.stopMotor();
    }
}