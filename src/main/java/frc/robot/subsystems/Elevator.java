package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.SwerveConstants.PID;

/* Will need to be convirted to a PID system */
public class Elevator extends PIDSubsystem {

    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;

    private final double upperLimit = 0.0;
    private final double lowerLimit = 0.0;

    public Elevator(int leftMotorID, int rightMotorID) {
        super(new PIDController(PID.Elevator.kP, PID.Elevator.kI, PID.Elevator.kD));
        this.getController().setTolerance(PID.Elevator.kT);
        
        leftMotor = new CANSparkMax(leftMotorID, MotorType.kBrushless);
        rightMotor = new CANSparkMax(rightMotorID, MotorType.kBrushless);

        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();
        
        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);
    }

    public void useOutput(double output, double setpoint) {
        if (setpoint > upperLimit) {
            setpoint = upperLimit;
            this.setSetpoint(upperLimit);
            leftMotor.set(0.0);
            rightMotor.set(0.0);
            return;
        } else if (setpoint < lowerLimit) {
            setpoint = lowerLimit;
            this.setSetpoint(lowerLimit);
            leftMotor.set(0.0);
            rightMotor.set(0.0);
            return;
        }
        leftMotor.set(output);
        rightMotor.set(-output);
    }

    @Override
    public double getMeasurement() {
        return leftMotor.getEncoder().getPosition();
    }

    /** Run approx. every 20 ms. */
    @Override
    public void periodic() {
        super.periodic();
    }
}
