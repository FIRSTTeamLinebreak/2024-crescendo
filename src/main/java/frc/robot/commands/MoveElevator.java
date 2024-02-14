package frc.robot.commands;

import static frc.robot.Util.applyCircularDeadZone;
import static frc.robot.Util.applyLinearDeadZone;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.JoystickConstants;
import frc.robot.subsystems.Elevator;

public class MoveElevator extends Command {

    private final double setpoint;
    private final Elevator m_elevator;

    public MoveElevator(Elevator m_elevator, double setpoint) {
        this.setpoint = setpoint;
        this.m_elevator = m_elevator;

        addRequirements(m_elevator);
    }

    /** Called once when the command is initially scheduled. */
    @Override
    public void initialize() {
        m_elevator.setPoint(setpoint);
    }

    /** Called repeatedly while the command is scheduled. */
    @Override
    public void execute() {}

    /**
     * Called when either the command finishes normally, or when it interrupted/canceled. Do not
     * schedule commands here that share requirements with this command. Use andThen(Command)
     * instead.
     *
     * @param interrupted Weather this command was interrupted
     */
    @Override
    public void end(boolean interrupted) {}

    /**
     * Whether the command has finished. If true, calls end() and stops the command from executing
     *
     * @return boolean
     */
    @Override
    public boolean isFinished() {
        SmartDashboard.putBoolean("Move Elevator Finished?", m_elevator.atSetpoint());
        return m_elevator.atSetpoint();
    }
}
