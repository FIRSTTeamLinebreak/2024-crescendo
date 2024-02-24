package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Vision;
public class visionLauncherRotation extends Command {

    private final double minLauncherSetpoint = .77;
    private final double maxLauncherSetpoint = .88;
    private final Launcher m_launcher;
    private final Vision m_vision;
    private final Elevator m_elevator;
    private final LinearFilter filter;
    private double measurement = 0;

    private double setpoint;

    public visionLauncherRotation(Launcher m_launcher, Vision m_vision, Elevator m_elevator) {
        this.m_launcher = m_launcher;        
        this.m_vision = m_vision;
        this.m_elevator = m_elevator;

        filter = LinearFilter.movingAverage(10);

        addRequirements(m_launcher);
        addRequirements(m_vision);
    }

    /** Called once when the command is initially scheduled. */
    @Override
    public void initialize() {}

    /** Called repeatedly while the command is scheduled. */
    @Override
    public void execute() {
        // double slope = (maxLauncherSetpoint - minLauncherSetpoint) / 1.6;
        // setpoint = (filter.calculate(m_vision.getLengthToBase()) * slope + minLauncherSetpoint);
        // m_launcher.setRotationSetpoint(setpoint);
        
        if(m_vision.lastTagSeen() == 3) {
            double slope = ((minLauncherSetpoint - maxLauncherSetpoint) / (2.8 - 1.2));
            if(m_vision.getAprilTagID() != -1) {
                measurement = filter.calculate(m_vision.getLengthToBase());
            }
            m_launcher.setRotationSetpoint(.15 + (maxLauncherSetpoint + (measurement + 1.2) * slope));
            m_elevator.moveToSetpoint(10);
        }
        else if(m_vision.lastTagSeen() == 5) {
            m_elevator.moveToSetpoint(100).andThen(m_launcher.moveClawToSetpoint(.48));
        }

        else if(m_vision.lastTagSeen() == 11) {
            m_elevator.moveToSetpoint(48).andThen(m_launcher.moveClawToSetpoint(.905));
        }

        // else if(m_vision.lastTagSeen() == 5) {
        //     m_launcher.setRotationSetpoint(.27);
        // }
        // else {
        //     m_launcher.setRotationSetpoint(1.0);
        // }
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
