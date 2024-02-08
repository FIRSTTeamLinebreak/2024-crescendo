package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    private double launcherSpeed = 0.0;
    private double controlSpeed = 0.0;
    private double rotationSetpoint = 0.0;

    private double upperLimit = 0.0;
    private double lowerLimit = 0.0;

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

        rotationEncoder = new DutyCycleEncoder(9);

        rotationPID = new PIDController(0.1, 0.0, 0.0);
        rotationPID.setTolerance(0.1);
    }

    public void setLauncherSpeed(double speed) {
        launcherSpeed = speed;
    }

    public void setControlSpeed(double speed) {
        controlSpeed = speed;
    }

    public void setRotationSetpoint(double setpoint) {
        rotationSetpoint = setpoint;
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

        if (rotationPIDEnabled) {
            if (rotationSetpoint > upperLimit) {
                rotationSetpoint = upperLimit;
                launcherRotation.set(0.0);
                return;
            } else if (rotationSetpoint < lowerLimit) {
                rotationSetpoint = lowerLimit;
                launcherRotation.set(0.0);
                return;
            }
            launcherRotation.set(rotationPID.calculate(rotationEncoder.get(), rotationSetpoint));
        } else {
            launcherRotation.set(0.0);
        }
    }
}
