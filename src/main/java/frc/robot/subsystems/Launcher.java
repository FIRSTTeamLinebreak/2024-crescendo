package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    private final double encoderOffest = 0.4;

    private double launcherSpeed = 0.0;
    private double controlSpeed = 0.0;
    private double rotationSetpoint;

    private double upperLimit = 1.0;
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

        flyWheelLeader.setSmartCurrentLimit(20);
        flyWheelFollower.setSmartCurrentLimit(20);
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
        // .56 to .88 1.4 rotations
        // -0.44 to 0.88
        double measurement = rotationEncoder.get();
        return 1 - (measurement - encoderOffest) * 2;
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

    public void enableRotationPID() {
        rotationPIDEnabled = true;
    }

    public void disableRotationPID() {
        rotationPIDEnabled = false;
    }

    /** Run approx. every 20 ms. */
    @Override
    public void periodic() {
        flyWheelLeader.set(launcherSpeed);
        flyWheelFollower.set(-launcherSpeed);
        controlWheels.set(controlSpeed);

        SmartDashboard.putNumber("Claw Rotation", getMeasurement());
        SmartDashboard.putNumber("Claw Setpoint", rotationSetpoint);

        if (rotationPIDEnabled) {
            double calculatedSpeed = rotationPID.calculate(getMeasurement(), rotationSetpoint);
            double clampedSpeed = MathUtil.clamp(calculatedSpeed, -0.5, 0.5);
            double feedForward = Math.abs(Math.sin(getMeasurement() * Math.PI) * PID.ClawRotation.kFF);
            launcherRotation.set(clampedSpeed + feedForward);
            // launcherRotation.set(feedForward);
        } else {
            launcherRotation.set(0.0);
        }
    }

    public Command moveClawToSetpoint(double setpoint) {
        return new InstantCommand(() -> this.setRotationSetpoint(setpoint)).repeatedly().until(this::rotationAtSetpoint);
    }
}
