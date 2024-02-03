package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final CANSparkMax intakeMotor;

    private double intakeSpeed;
    private double intakeSpeedTarget = 0.3;

    public Intake(int intakeMotorID) {
        intakeMotor = new CANSparkMax(intakeMotorID, MotorType.kBrushless);

        intakeMotor.restoreFactoryDefaults();
        
        intakeMotor.setIdleMode(IdleMode.kCoast);
    }

    public void toggleFloorIntake() {
        if (intakeSpeed == 0.0) {
            intakeSpeed = intakeSpeedTarget;
        }
        else {
            intakeSpeed = 0.0;
        }
    }


    // public void holdIntake() {
    //     intakeSpeed = intakeSpeedTarget;
    // }

    // public void releaseIntake() {
    //     intakeSpeed = 0;
    // }

    /** Run approx. every 20 ms. */
    @Override
    public void periodic() {
        intakeMotor.set(intakeSpeed);
    }
}
