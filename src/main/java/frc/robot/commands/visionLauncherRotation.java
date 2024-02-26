package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.Vision;
public class visionLauncherRotation extends Command {

    private final double minLauncherSetpoint = .85;
    private final double maxLauncherSetpoint = .9;
    private final Launcher m_launcher;
    private final Vision m_vision;
    private final Elevator m_elevator;
    private final LinearFilter filter;
    // private final StateMachine stateMachine;
    private double measurement = 0;

    private double setpoint;

    public visionLauncherRotation(Launcher m_launcher, Vision m_vision, Elevator m_elevator) {
        this.m_launcher = m_launcher;        
        this.m_vision = m_vision;
        this.m_elevator = m_elevator;

        filter = LinearFilter.movingAverage(10);
        // stateMachine = new StateMachine(Launcher m_launcher, Elevator m_elevator, Vision m_vision);

        addRequirements(m_launcher);
        addRequirements(m_vision);
    }

    /** Called once when the command is initially scheduled. */
    @Override
    public void initialize() {
        // if(stateMachine.getState())
    }

    /** Called repeatedly while the command is scheduled. */
    @Override
    public void execute() {
        // double slope = (maxLauncherSetpoint - minLauncherSetpoint) / 1.6;
        // setpoint = (filter.calculate(m_vision.getLengthToBase()) * slope + minLauncherSetpoint);
        // m_launcher.setRotationSetpoint(setpoint);
        
        if(m_vision.lastTagSeen() == 3 || m_vision.lastTagSeen() == 4) {
            // double slope = ((minLauncherSetpoint - maxLauncherSetpoint) / (2.8 - 1.2));
            if(m_vision.getAprilTagID() != -1) {
                measurement = filter.calculate(m_vision.getLengthToBase());
                double slope = Math.sqrt((2 * 9.81 * (measurement + 1.2)));
                // m_launcher.setRotationSetpoint(.17 + (maxLauncherSetpoint + (measurement + 1.2) * slope));
                m_launcher.setRotationSetpoint(maxLauncherSetpoint + (maxLauncherSetpoint * slope));
                // velocity Y = sqrt(2 * 9.81 * deltaY)
                // velocity y / delta Y = time
                // length to base /  time = velocity x
                // sqrt(vx^2 + vy^2) = velocity
                // velocity bvASZXgj
            }
            
        }
        else if(m_vision.lastTagSeen() == 5) {
            m_elevator.moveToSetpoint(100).andThen(m_launcher.moveClawToSetpoint(.48));
        }

        else if(m_vision.lastTagSeen() == 11) {
            m_elevator.moveToSetpoint(48).andThen(m_launcher.moveClawToSetpoint(.905));
        }

        else if(m_vision.lastTagSeen() == 5) {
            m_launcher.setRotationSetpoint(.27);
        }
        else {
            m_launcher.setRotationSetpoint(1.0);
        }

        SmartDashboard.putNumber("Last April Tag Seen", m_vision.lastTagSeen());
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
