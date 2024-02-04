package frc.robot;

import static frc.robot.Util.applyLinearDeadZone;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.JoystickDriveCommand;
import frc.robot.commands.SetLEDRed;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final SwerveDrive m_swerveDrive;
    private final Vision m_vision;
    // private final LED m_led = new LED(0, 10);
    // private final Launcher m_launcher = new Launcher(1, 2, 3, 4);
    // private final Intake m_intake = new Intake(5);
    // private final Elevator m_elevator = new Elevator(6, 7);

    private final PIDController visionPID;
    private final SendableChooser<Command> autoChooser;

    private final CommandXboxController m_driverController;
    private final JoystickDriveCommand m_SwerveDriveCommand;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        m_swerveDrive = new SwerveDrive();
        m_vision = new Vision("Camera_Module_v1");

        visionPID = new PIDController(0.02, 0, 0);

        m_driverController = new CommandXboxController(JoystickConstants.driveControllerId);
        m_SwerveDriveCommand = new JoystickDriveCommand(
                    m_swerveDrive,
                    () -> m_driverController.getLeftX(),
                    () -> m_driverController.getLeftY() * -1,
                    () -> {
                        // Rotation
                        double angle = m_vision.getTargetAngle();
                        if (m_driverController.getHID().getAButton() && m_vision.hasTargets()) {
                            return visionPID.calculate(angle, 0);
                        }

                        return applyLinearDeadZone(
                                        JoystickConstants.joystickDeadZone,
                                        m_driverController.getRightX() * -1)
                                / (m_driverController.getHID().getLeftBumper()
                                        ? JoystickConstants.slowTurningDivisor
                                        : 1);
                    },
                    () -> !m_driverController.getHID().getRightBumper());
        // new SetLEDRed(m_led).schedule();

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser("FirstAuto");

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        SmartDashboard.putData("Auto Chooser", autoChooser);   
        System.out.println(SmartDashboard.getData("Auto Chooser"));
    }


    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
  public Command getAutonomousCommand() {
        PathPlannerPath path = PathPlannerPath.fromPathFile("Test Path");
        
        System.out.println("Auto command");
        return AutoBuilder.followPath(path);
  }

  public void initTeleop() {
        m_swerveDrive.setDefaultCommand(m_SwerveDriveCommand);
        // m_driverController.a().onTrue(new InstantCommand(m_launcher::toggleLauncherIntake, m_launcher));
        // m_driverController.a().whileTrue(new InstantCommand(m_intake::holdIntake(), m_intake));
        // m_driverController.a().onFalse(new InstantCommand(m_intake::releaseIntake(), m_intake));
        // m_driverController.b().onTrue(new InstantCommand(m_launcher::toggleLauncherOutake, m_launcher));
  }

}
