package frc.robot;

import static frc.robot.Util.applyLinearDeadZone;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.AutoNoteLaunch;
import frc.robot.commands.JoystickDriveCommand;
import frc.robot.commands.visionLauncherRotation;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;
// import frc.robot.subsystems.LED;

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
    private final Odometry m_odometry;
    private final Vision m_vision;
    // private final LED m_led;
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
        m_vision = new Vision();
        m_swerveDrive = new SwerveDrive(m_vision);
        m_odometry = m_swerveDrive.getOdometry();
        // m_led = new LED(0, 10);
        m_launcher = new Launcher(12, 13, 11, 14);
        m_elevator = new Elevator(6, 7);
        m_elevator.disable();
        m_intake = new Intake(5);

        visionPID = new PIDController(0.02, 0, 0);

        m_driveController = new CommandXboxController(JoystickConstants.driveControllerId);
        m_scoreController = new CommandXboxController(JoystickConstants.scoreControllerId);
        m_SwerveDriveCommand = new JoystickDriveCommand(m_swerveDrive,
                () -> {
                    return m_driveController.getLeftY() 
                            / (m_driveController.x().getAsBoolean() ? 0.25 : 1.0);
                }, () -> {
                    return m_driveController.getLeftX() 
                            / (m_driveController.x().getAsBoolean() ? 0.25 : 1.0);
                }, () -> {
                    // Rotation
                    double angle = m_odometry.getRobotPose().getRotation().getRadians();
                    SmartDashboard.putNumber("angle", angle);
                    SmartDashboard.putNumber("triggeraxis", m_driveController.getHID().getRightTriggerAxis());
                    if (m_driveController.getHID().getRightTriggerAxis() > 0.5 && m_vision.lastTagSeen() == 4) {
                        SmartDashboard.putBoolean("speaker alignment", true);
                        return visionPID.calculate(angle, Math.tanh((m_odometry.getRobotPose().getY() - 8.308467)
                                / (m_odometry.getRobotPose().getX() - 1.442593)));
                    } else if (m_driveController.getHID().getRightTriggerAxis() > 0.5 && m_vision.lastTagSeen() == 5) {
                        SmartDashboard.putBoolean("amp alignment", true);
                        return visionPID.calculate(angle, 0);
                    } else if (m_driveController.getHID().getRightTriggerAxis() > 0.5 && m_vision.lastTagSeen() == 11) {
                        SmartDashboard.putBoolean("amp alignment", true);
                        return visionPID.calculate(angle, 0);
                    }

                    return applyLinearDeadZone(
                            JoystickConstants.joystickDeadZone, m_driveController.getRightX());
                }, () -> !m_driveController.getHID().getRightBumper());

        NamedCommands.registerCommand("AutoNoteLaunch",
                new AutoNoteLaunch(m_swerveDrive, m_elevator, m_launcher, m_vision));
        NamedCommands.registerCommand("StartAutoIntake", (m_elevator.moveToSetpoint(50)
                .alongWith(m_launcher.moveClawToSetpoint(0.5).withTimeout(1))
                .andThen(m_launcher.moveClawToSetpoint(0.07))
                .andThen(m_elevator.moveToSetpoint(37))).alongWith(new InstantCommand(() -> {
                    m_launcher.setLauncherSpeed(0.37);
                    m_launcher.setControlSpeed(0.1);
                    m_intake.setSpeed(0.45);
                })));
        NamedCommands.registerCommand("AutoStow", m_elevator.moveToSetpoint(50)
                .andThen(m_launcher.moveClawToSetpoint(1.0)
                        .alongWith(m_elevator.moveToSetpoint(10)))
                .alongWith(
                        new InstantCommand(() -> {
                            m_launcher.setLauncherSpeed(0.0);
                            m_launcher.setControlSpeed(-0.01);
                            m_intake.setSpeed(0.0);
                        })));

        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            SmartDashboard.putString("PP Cur Pos", pose.toString());
        });

        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            SmartDashboard.putString("PP Tgt Pos", pose.toString());
        });

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        m_launcher.enableRotationPID();
        m_elevator.enable();
        m_launcher.setControlSpeed(0.05);
        return autoChooser.getSelected();
    }

    public void initTeleop() {
        CommandScheduler scheduler = CommandScheduler.getInstance();

        m_swerveDrive.setDefaultCommand(m_SwerveDriveCommand);

        m_launcher.setRotationSetpoint(m_launcher.getMeasurement());
        m_elevator.setPoint(m_elevator.getMeasurement());

        m_launcher.enableRotationPID();
        m_elevator.enable();

        m_scoreController.povDown().onTrue(new InstantCommand(scheduler::cancelAll));
        m_scoreController.b().onTrue(m_launcher.launchCommand(m_vision.lastTagSeen()));

        // Intake Command
        m_scoreController.leftTrigger().onTrue(
                m_elevator.moveToSetpoint(50)
                        .alongWith(m_launcher.moveClawToSetpoint(0.60).withTimeout(0.75))
                        .andThen(m_launcher.moveClawToSetpoint(0.05).withTimeout(0.5))
                        .andThen(m_elevator.moveToSetpoint(35))
                        .andThen(new InstantCommand(() -> {
                            m_launcher.setLauncherSpeed(0.47);
                            m_launcher.setControlSpeed(0.1);
                            m_intake.setSpeed(0.45);
                        })));

        // Enable Intake Weeks
        m_scoreController.leftBumper().onTrue(new InstantCommand(() -> {
            m_launcher.setLauncherSpeed(0.37);
            m_launcher.setControlSpeed(0.1);
            m_intake.setSpeed(0.45);
        }));

        // Launcher Command
        m_scoreController.rightTrigger().whileTrue(
                new visionLauncherRotation(m_launcher, m_elevator, m_odometry, m_vision, true));

        // stow
        m_scoreController.rightBumper().onTrue(
                m_elevator.moveToSetpoint(50)
                        .andThen(m_launcher.moveClawToSetpoint(0.98).withTimeout(0.5))
                        .andThen(m_elevator.moveToSetpoint(10))
                        .alongWith(
                                new InstantCommand(() -> {
                                    m_launcher.setLauncherSpeed(0.0);
                                    m_launcher.setControlSpeed(0.05);
                                    m_intake.setSpeed(0.0);
                                })));

        m_scoreController.povUp().onTrue(m_elevator.moveToSetpoint(50).andThen(
                m_launcher.moveClawToSetpoint(0.5).withTimeout(1),
                m_elevator.moveToSetpoint(100)));

        Command elevatorCommand = new InstantCommand(() -> {
            double joystickValue = applyLinearDeadZone(JoystickConstants.joystickDeadZone, m_scoreController.getLeftY())
                    * 3;
            if (joystickValue != 0.0) {
                m_elevator.setPoint(m_elevator.getMeasurement() - joystickValue);
            }
        }).repeatedly();
        elevatorCommand.addRequirements(m_elevator);
        m_elevator.setDefaultCommand(elevatorCommand);

        Command launcherCommand = new InstantCommand(() -> {
            double joystickValue = applyLinearDeadZone(JoystickConstants.joystickDeadZone,
                    m_scoreController.getRightY()) * 0.1;
            if (joystickValue != 0.0) {
                m_launcher.setRotationSetpoint(m_launcher.getMeasurement() - joystickValue);
            }
        }).repeatedly();
        launcherCommand.addRequirements(m_launcher);
        m_launcher.setDefaultCommand(launcherCommand);
    }

    public void initTest() {
        m_scoreController.a().onTrue(new InstantCommand(() -> m_launcher.setRotationSetpoint(0.25)));
        m_scoreController.b().onTrue(new InstantCommand(() -> m_launcher.setRotationSetpoint(0.50)));
        m_scoreController.y().onTrue(new InstantCommand(() -> m_launcher.setRotationSetpoint(0.75)));
        // m_scoreController.a().onTrue(new InstantCommand(() ->
        // m_elevator.setPoint(10.0)));
        // m_scoreController.b().onTrue(new InstantCommand(() ->
        // m_elevator.setPoint(50.0)));
        // m_scoreController.y().onTrue(new InstantCommand(() ->
        // m_elevator.setPoint(75.0)));

        m_scoreController.x().onTrue(new InstantCommand(m_elevator::enable, m_elevator));
        m_scoreController.x().onTrue(new InstantCommand(m_launcher::enableRotationPID, m_launcher));
        m_scoreController.x().onFalse(new InstantCommand(m_elevator::disable, m_elevator));
        m_scoreController.x().onFalse(new InstantCommand(m_launcher::disableRotationPID, m_launcher));

        Command elevatorCommand = new InstantCommand(() -> {
            double joystickValue = applyLinearDeadZone(JoystickConstants.joystickDeadZone,
                    m_scoreController.getLeftY() * 2);
            if (joystickValue != 0.0) {
                m_elevator.setPoint(m_elevator.getMeasurement() + joystickValue);
            }
        }).repeatedly();
        elevatorCommand.addRequirements(m_elevator);
        m_elevator.setDefaultCommand(elevatorCommand);

        Command launcherCommand = new InstantCommand(() -> {
            double joystickValue = applyLinearDeadZone(JoystickConstants.joystickDeadZone,
                    m_scoreController.getRightY()) * 0.2;
            if (joystickValue != 0.0) {
                m_launcher.setRotationSetpoint(m_launcher.getMeasurement() + joystickValue);
            }
        }).repeatedly();
        launcherCommand.addRequirements(m_launcher);
        m_launcher.setDefaultCommand(launcherCommand);
    }

    public void disable() {
        m_launcher.disableRotationPID();
        m_elevator.disable();
    }
}
