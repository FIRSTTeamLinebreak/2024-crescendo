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
        // if(m_vision.getAprilTagID() != -1) {
        //     measurement = filter.calculate(m_vision.getLengthToBase());
        //     double slope = Math.sqrt((2 * 9.81 * (measurement + 1.2))) * -1;
        //     double slope = ((minLauncherSetpoint - maxLauncherSetpoint) / (2.8 - 1.2));
        //     m_launcher.setRotationSetpoint(.17 + (maxLauncherSetpoint + (measurement + 1.2) * slope));
        //     m_launcher.setRotationSetpoint(maxLauncherSetpoint + (maxLauncherSetpoint * slope));
        // }
        if(m_vision.lastTagSeen() == 3 || m_vision.lastTagSeen() == 4) {
            if(m_vision.getAprilTagID() != -1) {
                measurement = filter.calculate(m_vision.getLengthToBase());
                // double X = Math.tanh(1.076325 / (measurement - .1778)) * 57.3248;
                // double Y = (((99.048 - (3.3042 * X) + (.0713 * X * X) - (.00035 * X * X * X)) * .0174444) / Math.PI) + .5;
                double angle = (Math.tanh(1.73 / (measurement - .51)) + (Math.PI / 2)) / Math.PI;
                if(angle < .5){
                    m_launcher.setRotationSetpoint(.5);
                }
                else if(angle > 1.0) {
                    m_launcher.setRotationSetpoint(.95);
                }
                else {
                    m_launcher.setRotationSetpoint(angle);
                }
            }
        }

        // amp
        else if(m_vision.lastTagSeen() == 5 || m_vision.lastTagSeen() == 6) {
            m_elevator.moveToSetpoint(100)
            .alongWith(m_launcher.moveClawToSetpoint(0.27)).schedule();
        }

        // trap
        else if(m_vision.lastTagSeen() == 11 || m_vision.lastTagSeen() == 12 || m_vision.lastTagSeen() == 13 || m_vision.lastTagSeen() == 14 || m_vision.lastTagSeen() == 15 || m_vision.lastTagSeen() == 16) {
            m_elevator.moveToSetpoint(48)
            .andThen(m_launcher.moveClawToSetpoint(.905)).schedule();
        }
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
