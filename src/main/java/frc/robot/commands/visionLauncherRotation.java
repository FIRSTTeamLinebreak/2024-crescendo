package frc.robot.commands;

import static frc.robot.Util.isAmp;
import static frc.robot.Util.isSpeaker;
import static frc.robot.Util.isTrap;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.Vision;

public class visionLauncherRotation extends Command {
    private final Launcher m_launcher;
    private final Elevator m_elevator;
    private final Odometry m_odometry;
    private final Vision m_vision;
    private final boolean stow;

    private int lastTagSeen = 0;

    public visionLauncherRotation(Launcher m_launcher, Elevator m_elevator, Odometry m_odometry, Vision m_vision,
            boolean stow) {
        this.m_launcher = m_launcher;
        this.m_elevator = m_elevator;
        this.m_odometry = m_odometry;
        this.m_vision = m_vision;
        this.stow = stow;

        addRequirements(m_launcher, m_elevator, m_odometry, m_vision);
    }

    /** Called once when the command is initially scheduled. */
    @Override
    public void initialize() {
        this.lastTagSeen = m_vision.lastTagSeen();
    }

    /** Called repeatedly while the command is scheduled. */
    @Override
    public void execute() {
        if (isSpeaker(lastTagSeen) && m_vision.hasTargets()) {
            Pose2d pose = m_odometry.getRobotPose();
            Translation2d speakerPose = m_odometry.getAlliance()
                    ? new Translation2d(16.57, 5.547868)
                    : new Translation2d(0.0, 5.547868);
            double measurement = pose.getTranslation().getDistance(speakerPose);
            SmartDashboard.putNumber("Distance", measurement);
            double angle = (Math.tanh(1.71 / (measurement - 0.31)) + (Math.PI / 2)) / Math.PI;

            m_launcher.setRotationSetpoint(MathUtil.clamp(angle, 0.5, 1.0));
        }

        else if (isAmp(lastTagSeen)) {
            m_elevator.setPoint(100);
            m_launcher.setRotationSetpoint(0.27);
        }

        else if (isTrap(lastTagSeen)) {
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
        if (stow) {
            if (isSpeaker(lastTagSeen)) {
                m_launcher.setRotationSetpoint(1.0);
                return;
            }
            m_launcher.setRotationSetpoint(1.0);
            m_elevator.setPoint(10);
        }
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
