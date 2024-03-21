package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

public class AutoNoteLaunch extends SequentialCommandGroup {

    private final SwerveDrive m_swerveDrive;
    private final Elevator m_elevator;
    private final Launcher m_launcher;
    private final Vision m_vision;
    
    public AutoNoteLaunch(SwerveDrive swerveDrive, Elevator elevator, Launcher launcher, Vision vision) {
        m_swerveDrive = swerveDrive;
        m_elevator = elevator;
        m_launcher = launcher;
        m_vision = vision;

        addCommands(
            new visionLauncherRotation(m_launcher, m_elevator, m_swerveDrive.getOdometry(), m_vision, false).withTimeout(1),
            m_launcher.launchCommand(m_vision.lastTagSeen())
        );

        addRequirements(m_elevator, m_launcher, m_swerveDrive, m_swerveDrive.getOdometry(), m_vision);
    }

}
