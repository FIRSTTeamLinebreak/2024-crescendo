package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;

    private double elevatorSpeed = 0.0;
    private double elevatorSpeedTarget = 0.1;

    public Elevator(int leftMotorID, int rightMotorID) {
        leftMotor = new CANSparkMax(leftMotorID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(rightMotorID, MotorType.kBrushless)

        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();
        
        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);
    }

    public void moveElevator() {
        if (elevatorSpeed == 0.0) {
            elevatorSpeed = elevatorSpeedTarget;
        }
        else {
            elevatorSpeed = 0.0;
        }
    }


    /** Run approx. every 20 ms. */
    @Override
    public void periodic() {
        leftMotor.set(elevatorSpeed);
        rightMotor.set(-elevatorSpeed);
    }
}