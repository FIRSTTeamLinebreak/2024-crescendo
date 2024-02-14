package frc.robot.commands;

import static frc.robot.Util.applyCircularDeadZone;
import static frc.robot.Util.applyLinearDeadZone;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.JoystickConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Launcher;

public class MoveLauncherRotation extends Command {

    private final double setpoint;
    private final Launcher m_launcher;

    public MoveLauncherRotation(Launcher m_launcher, double setpoint) {
        this.setpoint = setpoint;
        this.m_launcher = m_launcher;

        addRequirements(m_launcher);
    }

    /** Called once when the command is initially scheduled. */
    @Override
    public void initialize() {
        m_launcher.setRotationSetpoint(setpoint);
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
        return m_launcher.rotationAtSetpoint();
    }
}
