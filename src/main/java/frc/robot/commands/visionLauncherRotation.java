package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Vision;

public class visionLauncherRotation extends Command {

    // private final double minLauncherSetpoint = .85;
    // private final double maxLauncherSetpoint = .9;
    private final Launcher m_launcher;
    private final Elevator m_elevator;
    private final Vision m_vision;
    private final LinearFilter filter;
    private double lastTagSeen = 0.0;

    public visionLauncherRotation(Launcher m_launcher, Vision m_vision, Elevator m_elevator) {
        this.m_launcher = m_launcher;
        this.m_elevator = m_elevator;
        this.m_vision = m_vision;

        filter = LinearFilter.movingAverage(10);

        addRequirements(m_launcher, m_elevator, m_vision);
    }

    /** Called once when the command is initially scheduled. */
    @Override
    public void initialize() {
        this.lastTagSeen = m_vision.lastTagSeen();
    }

    /** Called repeatedly while the command is scheduled. */
    @Override
    public void execute() {
        if (((lastTagSeen == 3 || lastTagSeen == 4) // Red Speaker
                || (lastTagSeen == 7 || lastTagSeen == 8)) // Blue Speaker
                && m_vision.hasTargets()) {
            double measurement = filter.calculate(m_vision.getLengthToBase());
            double angle = (Math.tanh(1.71 / (measurement - .31)) + (Math.PI / 2)) / Math.PI;

            m_launcher.setRotationSetpoint(MathUtil.clamp(angle, 0.5, 1.0));
        }

        else if (((lastTagSeen == 5) // Red Amp
                || (lastTagSeen == 6)) // Blue Amp
        ) {
            m_elevator.setPoint(100);
            m_launcher.setRotationSetpoint(0.27);
        }

        else if ((lastTagSeen == 11 || lastTagSeen == 12 || lastTagSeen == 13) // Red Trap
                || (lastTagSeen == 14 || lastTagSeen == 15 || lastTagSeen == 16) // Blue Trap
        ) {
            m_elevator.setPoint(48);
            m_launcher.setRotationSetpoint(.905);
        }
    }

    /**
     * Called when either the command finishes normally, or when it
     * interrupted/canceled. Do not
     * schedule commands here that share requirements with this command. Use
     * andThen(Command)
     * instead.
     *
     * @param interrupted Weather this command was interrupted
     */
    @Override
    public void end(boolean interrupted) {
        // Returning from Speaker
        if(lastTagSeen == 3 || lastTagSeen == 4 || lastTagSeen == 7 || lastTagSeen == 8) {
            m_launcher.setRotationSetpoint(1.0);
            return;
        }
        m_launcher.setRotationSetpoint(1.0);
        m_elevator.setPoint(10);
    }

    /**
     * Whether the command has finished. If true, calls end() and stops the command
     * from executing
     *
     * @return boolean
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
