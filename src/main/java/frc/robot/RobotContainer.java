package frc.robot;

import static frc.robot.Util.applyLinearDeadZone;

import com.pathplanner.lib.auto.AutoBuilder;
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
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final SwerveDrive m_swerveDrive;
    private final Vision m_vision;
    private final LED m_led;
    private final Launcher m_launcher;
    private final Elevator m_elevator;
    private final Intake m_intake;

    private final PIDController visionPID;
    private final SendableChooser<Command> autoChooser;

    private final CommandXboxController m_driveController;
    private final CommandXboxController m_scoreController;
    private final JoystickDriveCommand m_SwerveDriveCommand;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        m_swerveDrive = new SwerveDrive();
        m_vision = new Vision();
        m_led = new LED(0, 10);
        m_launcher = new Launcher(12, 13, 11, 14);
        m_elevator = new Elevator(6, 7);
        m_elevator.disable();
        m_intake = new Intake(5);

        visionPID = new PIDController(0.02, 0, 0);

        m_driveController = new CommandXboxController(JoystickConstants.driveControllerId);
        m_scoreController = new CommandXboxController(JoystickConstants.scoreControllerId);
        m_SwerveDriveCommand = new JoystickDriveCommand(m_swerveDrive,
                () -> m_driveController.getLeftX(), () -> m_driveController.getLeftY() * -1, () -> {
                    // Rotation
                    double angle = m_vision.getTargetAngle();
                    if (m_driveController.getHID().getAButton() && m_vision.hasTargets()) {
                        return visionPID.calculate(angle, 0);
                    }

                    return applyLinearDeadZone(
                            JoystickConstants.joystickDeadZone, m_driveController.getRightX())
                            / (m_driveController.getHID().getLeftBumper()
                                    ? JoystickConstants.slowTurningDivisor
                                    : 1);
                }, () -> !m_driveController.getHID().getRightBumper());
        new SetLEDRed(m_led).schedule();

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser("FirstAuto");

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        SmartDashboard.putData("Auto Chooser", autoChooser);
        System.out.println(SmartDashboard.getData("Auto Chooser"));
    }

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
        m_scoreController.a().onTrue(new InstantCommand(() -> {
            m_launcher.setLauncherSpeed(0.3);
            m_launcher.setControlSpeed(0.1);
            m_intake.setSpeed(0.3);
        }));
        m_scoreController.a().onFalse(new InstantCommand(() -> {
            m_launcher.setLauncherSpeed(0.0);
            m_launcher.setControlSpeed(0.05);
            m_intake.setSpeed(0.0);
        }));
        m_scoreController.b().onTrue(
            new InstantCommand(() -> {
                m_launcher.setLauncherSpeed(-1.0);
            })
                .repeatedly()
                .withTimeout(0.25)
                .andThen(new InstantCommand(() -> {
                    m_launcher.setControlSpeed(-1.0);
                })));
        m_scoreController.b().onFalse(new InstantCommand(() -> {
            m_launcher.setLauncherSpeed(0.0);
            m_launcher.setControlSpeed(0.0);
        }));
        m_launcher.setRotationSetpoint(m_launcher.getMeasurement());
        m_elevator.setPoint(m_elevator.getMeasurement());
        Command ele50 = new InstantCommand(() -> m_elevator.setPoint(50)).repeatedly().until(m_elevator::atSetpoint);
        Command clawIntake = new InstantCommand(() -> m_launcher.setRotationSetpoint(0.01)).repeatedly().until(m_launcher::rotationAtSetpoint);
        Command eleIntake = new InstantCommand(() -> m_elevator.setPoint(37)).repeatedly().until(m_elevator::atSetpoint);
        // m_scoreController.a().onTrue(ele50.andThen(clawIntake).andThen(eleIntake));

        m_scoreController.x().onTrue(new InstantCommand(m_elevator::enable, m_elevator));
        m_scoreController.x().onTrue(new InstantCommand(m_launcher::enableRotationPID, m_launcher));
        m_scoreController.x().onFalse(new InstantCommand(m_elevator::disable, m_elevator));
        m_scoreController.x().onFalse(new InstantCommand(m_launcher::disableRotationPID, m_launcher));

        Command elevatorCommand = new InstantCommand(() -> {
            double joystickValue = applyLinearDeadZone(JoystickConstants.joystickDeadZone, m_scoreController.getLeftY())*1.5;
            if (joystickValue != 0.0) {
                m_elevator.setPoint(m_elevator.getMeasurement() + joystickValue);
            }
        }).repeatedly();
        elevatorCommand.addRequirements(m_elevator);
        m_elevator.setDefaultCommand(elevatorCommand);

        Command launcherCommand = new InstantCommand(() -> {
            double joystickValue = applyLinearDeadZone(JoystickConstants.joystickDeadZone, m_scoreController.getRightY())*0.1;
            if (joystickValue != 0.0) {
                m_launcher.setRotationSetpoint(m_launcher.getMeasurement() + joystickValue);
            }
        }).repeatedly();
        launcherCommand.addRequirements(m_launcher);
        m_launcher.setDefaultCommand(launcherCommand);
        new InstantCommand(() -> SmartDashboard.putNumber("Claw Mesurment", m_launcher.getMeasurement()));
    }

    public void initTest() {
        m_scoreController.a().onTrue(new InstantCommand(() -> m_launcher.setRotationSetpoint(0.25)));
        m_scoreController.b().onTrue(new InstantCommand(() -> m_launcher.setRotationSetpoint(0.50)));
        m_scoreController.y().onTrue(new InstantCommand(() -> m_launcher.setRotationSetpoint(0.75)));
        // m_scoreController.a().onTrue(new InstantCommand(() -> m_elevator.setPoint(0.0)));
        // m_scoreController.b().onTrue(new InstantCommand(() -> m_elevator.setPoint(50.0)));
        // m_scoreController.y().onTrue(new InstantCommand(() -> m_elevator.setPoint(75.0)));

        m_scoreController.x().onTrue(new InstantCommand(m_elevator::enable, m_elevator));
        m_scoreController.x().onTrue(new InstantCommand(m_launcher::enableRotationPID, m_launcher));
        m_scoreController.x().onFalse(new InstantCommand(m_elevator::disable, m_elevator));
        m_scoreController.x().onFalse(new InstantCommand(m_launcher::disableRotationPID, m_launcher));

        Command elevatorCommand = new InstantCommand(() -> {
            double joystickValue = applyLinearDeadZone(JoystickConstants.joystickDeadZone, m_scoreController.getLeftY());
            if (joystickValue != 0.0) {
                m_elevator.setPoint(m_elevator.getMeasurement() + joystickValue);
            }
        }).repeatedly();
        elevatorCommand.addRequirements(m_elevator);
        m_elevator.setDefaultCommand(elevatorCommand);

        Command launcherCommand = new InstantCommand(() -> {
            double joystickValue = applyLinearDeadZone(JoystickConstants.joystickDeadZone, m_scoreController.getRightY())*0.05;
            if (joystickValue != 0.0) {
                m_launcher.setRotationSetpoint(m_launcher.getMeasurement() + joystickValue);
            }
        }).repeatedly();
        launcherCommand.addRequirements(m_launcher);
        m_launcher.setDefaultCommand(launcherCommand);
        new InstantCommand(() -> SmartDashboard.putNumber("Claw Mesurment", m_launcher.getMeasurement()));
    }
}
