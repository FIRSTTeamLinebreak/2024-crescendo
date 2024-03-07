package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants.PID;

public class Elevator extends PIDSubsystem {

    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;

    private final double upperLimit = 100.0;
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

        this.setSetpoint(getMeasurement());
    }

    public void setPoint(double setpoint) {
        if (setpoint > upperLimit) {
            setpoint = upperLimit;
        } else if (setpoint < lowerLimit) {
            setpoint = lowerLimit;
        }
        this.setSetpoint(setpoint);
    }

    public boolean atSetpoint() {
        return this.getController().atSetpoint();
    }

    public Command moveToSetpoint(double setpoint) {
        return new InstantCommand(() -> this.setPoint(setpoint)).repeatedly().until(this::atSetpoint);
    }

    @Override
    public void useOutput(double output, double setpoint) {
        leftMotor.set(-output);
        rightMotor.set(output);
    }

    @Override
    public double getMeasurement() {
        return leftMotor.getEncoder().getPosition() * -1;
    }

    public boolean atPoint() {
        if(this.getMeasurement() + Constants.SwerveConstants.PID.Elevator.kT >= this.getSetpoint() && this.getMeasurement() - Constants.SwerveConstants.PID.Elevator.kT <= this.getSetpoint()) {
            return true;
        }
        return false;
    }
    /** Run approx. every 20 ms. */
    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Ele setpoint", this.getSetpoint());
        SmartDashboard.putNumber("Ele Measurement", getMeasurement());
        SmartDashboard.putBoolean("Ele atSetpoint", this.getController().atSetpoint());
    }
}
