package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.Util.isTrap;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants.PID;

/* May need to change to a PID system for launcherRotation
 *
 * Might be best the change the way that the delay works currently */
public class Launcher extends SubsystemBase {
    private final CANSparkMax flyWheelLeader;
    private final CANSparkMax flyWheelFollower;
    private final CANSparkMax controlWheels;
    private final CANSparkMax launcherRotation;

    private final PIDController rotationPID;
    private final DutyCycleEncoder rotationEncoder;
    private boolean rotationPIDEnabled = false;

    private final double encoderOffest = 0.766;

    private double launcherSpeed = 0.0;
    private double controlSpeed = 0.0;
    private double rotationSetpoint;

    private double upperLimit = 0.97;
    private double lowerLimit = 0.02;

    public Launcher(
            int flyWheelLeaderID,
            int flyWheelFollowerID,
            int controlWheelsID,
            int launcherRotationID) {
        flyWheelLeader = new CANSparkMax(flyWheelLeaderID, MotorType.kBrushless);
        flyWheelFollower = new CANSparkMax(flyWheelFollowerID, MotorType.kBrushless);
        controlWheels = new CANSparkMax(controlWheelsID, MotorType.kBrushless);
        launcherRotation = new CANSparkMax(launcherRotationID, MotorType.kBrushless);

        flyWheelLeader.restoreFactoryDefaults();
        flyWheelFollower.restoreFactoryDefaults();
        controlWheels.restoreFactoryDefaults();
        launcherRotation.restoreFactoryDefaults();

        flyWheelLeader.setIdleMode(IdleMode.kCoast);
        flyWheelFollower.setIdleMode(IdleMode.kCoast);
        controlWheels.setIdleMode(IdleMode.kBrake);
        launcherRotation.setIdleMode(IdleMode.kBrake);

        flyWheelLeader.setSmartCurrentLimit(30);
        flyWheelFollower.setSmartCurrentLimit(30);
        controlWheels.setSmartCurrentLimit(30);
        launcherRotation.setSmartCurrentLimit(40);

        rotationEncoder = new DutyCycleEncoder(0);
        rotationSetpoint = getMeasurement();

        rotationPID = new PIDController(PID.ClawRotation.kP, PID.ClawRotation.kI, PID.ClawRotation.kD);
        rotationPID.setTolerance(PID.ClawRotation.kT);
    }

    public void setLauncherSpeed(double speed) {
        launcherSpeed = speed;
    }

    public void setControlSpeed(double speed) {
        controlSpeed = speed;
    }

    public double getMeasurement() {
        double measurement = (1 - (rotationEncoder.get() - encoderOffest) * 2);
        double whole = (int)measurement;
        if (measurement >= 1.2) {
            return measurement - whole;
        }
        return measurement;
    }

    public void setRotationSetpoint(double setpoint) {
        if (setpoint > upperLimit) {
            rotationSetpoint = upperLimit;
            return;
        } else if (setpoint < lowerLimit) {
            rotationSetpoint = lowerLimit;
            return;
        }
        rotationSetpoint = setpoint;
    }

    public boolean rotationAtSetpoint() {
        return rotationPID.atSetpoint();
    }

    public boolean atPoint(double target) {
        if (this.getMeasurement() + Constants.SwerveConstants.PID.Elevator.kT >= target
                && this.getMeasurement() - Constants.SwerveConstants.PID.Elevator.kT <= target) {
            return true;
        }
        return false;
    }

    public void enableRotationPID() {
        rotationPIDEnabled = true;
    }

    public void disableRotationPID() {
        rotationPIDEnabled = false;
    }

    /** Run approx. every 20 ms. */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Claw Rotation", getMeasurement());
        if (!rotationEncoder.isConnected()) {
            launcherRotation.set(0.0);
            return;
        }
        flyWheelLeader.set(launcherSpeed);
        flyWheelFollower.set(-launcherSpeed);
        controlWheels.set(controlSpeed);

        if (rotationPIDEnabled) {
            double calculatedSpeed = rotationPID.calculate(getMeasurement(), rotationSetpoint);
            double clampedSpeed = MathUtil.clamp(calculatedSpeed, -0.5, 0.5);
            double feedForward = Math.abs(Math.sin(getMeasurement() * Math.PI) * PID.ClawRotation.kFF);
            if (this.getMeasurement() > upperLimit && (calculatedSpeed + feedForward) > 0.0) {
                launcherRotation.set(0.0);
                return;
            }
            launcherRotation.set(clampedSpeed + feedForward);
        } else {
            launcherRotation.set(0.0);
        }
    }

    public Command moveClawToSetpoint(double setpoint) {
        return new InstantCommand(() -> this.setRotationSetpoint(setpoint)).repeatedly()
                .until(this::rotationAtSetpoint);
    }

    public Command launchCommand(int lastTagSeen) {
        Command finishLaunch = new InstantCommand(() -> {
            this.setControlSpeed(0.0);
            this.setLauncherSpeed(0.0);
        });
        if (isTrap(lastTagSeen)) {
            System.out.println("Shooting Trap");
            return new InstantCommand(() -> {
                this.setLauncherSpeed(-0.19);
            }).repeatedly().withTimeout(0.5).andThen(new InstantCommand(() -> {
                this.setControlSpeed(-0.19);
            }).repeatedly().withTimeout(0.25), finishLaunch);
        }
        return new InstantCommand(() -> {
            this.setLauncherSpeed(-1.0);
        }).repeatedly().withTimeout(0.75).andThen(new InstantCommand(() -> {
            this.setControlSpeed(-1.0);
        }).repeatedly().withTimeout(0.25), finishLaunch);
    }
}
