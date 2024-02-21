package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Vision;
public class visionLauncherRotation extends Command {

    private final double minLauncherSetpoint = .77;
    private final double maxLauncherSetpoint = .88;
    private final Launcher m_launcher;
    private final Vision m_vision;
    private double setpoint;

    public visionLauncherRotation(Launcher m_launcher, Vision m_vision) {
        this.m_launcher = m_launcher;        
        this.m_vision = m_vision;

        addRequirements(m_launcher);
        addRequirements(m_vision);
    }

    /** Called once when the command is initially scheduled. */
    @Override
    public void initialize() {}

    /** Called repeatedly while the command is scheduled. */
    @Override
    public void execute() {
        double slope = (maxLauncherSetpoint - minLauncherSetpoint) / 1.6;
        setpoint = (m_vision.getLengthToBase() * slope) + minLauncherSetpoint;
        m_launcher.setRotationSetpoint(setpoint);
    }

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
        return false;
    }
}
