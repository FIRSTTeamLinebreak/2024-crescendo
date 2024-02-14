package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final CANSparkMax intakeMotor;

    private double intakeSpeed;

    public Intake(int intakeMotorID) {
        intakeMotor = new CANSparkMax(intakeMotorID, MotorType.kBrushless);

        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setIdleMode(IdleMode.kCoast);

        intakeSpeed = 0.0;
    }

    public void setSpeed(double targetSpeed) {
        intakeSpeed = targetSpeed;
    } 

    /** Run approx. every 20 ms. */
    @Override
    public void periodic() {
        intakeMotor.set(intakeSpeed);
    }
}
