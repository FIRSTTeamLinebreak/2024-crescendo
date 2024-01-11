package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {

    private final CANSparkMax launcherMotorLeader;
    private final CANSparkMax launcherMotorFollower;

    private double launcherSpeed = 0.0;
    private double launcherSpeedTarget = 0.3;

    public Launcher(int launcherMotorLeaderId, int launcherMotorFollowerId) {
        launcherMotorLeader = new CANSparkMax(launcherMotorLeaderId, MotorType.kBrushless);
        launcherMotorFollower = new CANSparkMax(launcherMotorFollowerId, MotorType.kBrushless);

        launcherMotorLeader.restoreFactoryDefaults();
        launcherMotorFollower.restoreFactoryDefaults();

        launcherMotorLeader.setIdleMode(IdleMode.kCoast);
        launcherMotorFollower.setIdleMode(IdleMode.kCoast);
    }

    public void toggleLauncher() {
        if (launcherSpeed == 0.0) {
            launcherSpeed = launcherSpeedTarget;
        }
        else {
            launcherSpeed = 0.0;
        }
        System.out.println("Launcher Toggled " + launcherSpeed);
    }

    /** Run approx. every 20 ms. */
    @Override
    public void periodic() {
        launcherMotorLeader.set(launcherSpeed);
        launcherMotorFollower.set(-launcherSpeed);
    }
}
