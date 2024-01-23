package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {

    private final CANSparkMax flyWheelLeader;
    private final CANSparkMax flyWheelFollower;
    private final CANSparkMax controlWheels;
    private final CANSparkMax launcherRotation;

    private double launcherSpeed = 0.0;
    private double launcherSpeedTarget = -1.0;
    private double intakeSpeed = 0.0;
    private double intakeSpeedTarget = 0.3;
    private double controlIntakeSpeedTarget = .05;
    private double launcherRotationSpeed = 0.0;
    private double launcherRotationSpeedTarget = 0.0;
    private int i = 0;

    public Launcher(int flyWheelLeaderID, int flyWheelFollowerID, int controlWheelsID, int launcherRotationID) {
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
    }

    public void toggleLauncherIntake() {
        if (launcherSpeed == 0.0) {
            launcherSpeed = intakeSpeedTarget;
            intakeSpeed = controlIntakeSpeedTarget;
        }
        else {
            launcherSpeed = 0.0;
            intakeSpeed = 0.0;
        }
    }

    public void toggleLauncherOutake() {
        if (launcherSpeed == 0.0) {
            launcherSpeed = launcherSpeedTarget;
            intakeSpeed = launcherSpeedTarget;
        }
        else {
            launcherSpeed = 0.0;
            intakeSpeed = 0.0;
        }
    }

    // public void toggleLauncherRotation() {
    //     if(.getPosition() < launcherDegreeTarget) {
    //         launcherRotationSpeed = launcherRotationSpeedTarget;
    //     }
    // }

    // public void toggleIntake() {
    //     if (launcherSpeed == 0.0) {
    //         launcherSpeed = intakeSpeedTarget;
    //     }
    //     else {
    //         launcherSpeed = 0.0;
    //     }
    // }

    /** Run approx. every 20 ms. */
    @Override
    public void periodic() {
        flyWheelLeader.set(launcherSpeed);
        flyWheelFollower.set(-launcherSpeed);
        if (launcherSpeed < 0 && i < 13) {
            controlWheels.set(0);
            i++;
            return;      
        }
        controlWheels.set(intakeSpeed);
        i = 0;

    }
}
