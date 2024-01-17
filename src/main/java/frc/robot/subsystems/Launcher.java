package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {

    private final CANSparkMax flyWheelLeader;
    private final CANSparkMax flyWheelFollower;
    private final CANSparkMax controlWheelLeader;
    private final CANSparkMax controlWheelFollower;

    private double launcherSpeed = 0.0;
    private double launcherSpeedTarget = -1.0;
    private double intakeSpeed = 0.0;
    private double intakeSpeedTarget = 0.3;
    private double controlIntakeSpeedTarget = .05;
    private int i = 0;

    public Launcher(int flyWheelLeaderID, int flyWheelFollowerID, int controlWheelLeaderID, int controlWheelFollowerID) {
        flyWheelLeader = new CANSparkMax(flyWheelLeaderID, MotorType.kBrushless);
        flyWheelFollower = new CANSparkMax(flyWheelFollowerID, MotorType.kBrushless);
        controlWheelLeader = new CANSparkMax(controlWheelLeaderID, MotorType.kBrushless);
        controlWheelFollower = new CANSparkMax(controlWheelFollowerID, MotorType.kBrushless);

        flyWheelLeader.restoreFactoryDefaults();
        flyWheelFollower.restoreFactoryDefaults();
        controlWheelLeader.restoreFactoryDefaults();
        controlWheelFollower.restoreFactoryDefaults();
        
        flyWheelLeader.setIdleMode(IdleMode.kCoast);
        flyWheelFollower.setIdleMode(IdleMode.kCoast);
        controlWheelLeader.setIdleMode(IdleMode.kCoast);
        controlWheelFollower.setIdleMode(IdleMode.kCoast);
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
        if (launcherSpeed < 0 && i < 50) {
            controlWheelLeader.set(0);
            controlWheelFollower.set(0);
            i++;
            return;      
        }
        controlWheelLeader.set(intakeSpeed);
        controlWheelFollower.set(-intakeSpeed);
        i = 0;

    }
}
