// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Util.applyLinearDeadZone;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OiConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SwerveJoystickDriveCommand;
import frc.robot.subsystems.ExampleSubsystem;
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
	private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
	private final SwerveDrive m_swerveDrive = new SwerveDrive();
	private final Vision m_vision = new Vision("OV5647");

	private final PIDController visionPID = new PIDController(0.02,0,0);

	// Replace with CommandPS4Controller or CommandJoystick if needed
	private final CommandXboxController m_driverController = new CommandXboxController(OiConstants.driveControllerId);

	private final SwerveJoystickDriveCommand m_SwerveDriveCommand = new SwerveJoystickDriveCommand(
			m_swerveDrive,
			() -> m_driverController.getLeftX() * -1,
			() -> m_driverController.getLeftY() * -1,
			() -> {
				// Rotation
				PhotonPipelineResult result = m_vision.getResult();
				if (m_driverController.getHID().getAButton()) {
					if (result.hasTargets()) {
						return visionPID.calculate(result.getBestTarget().getYaw(), 0);
					}
				}

				if (applyLinearDeadZone(OiConstants.joystickDeadZone, m_driverController.getRightX()) != 0) {
					if (m_driverController.getHID().getLeftBumper()) {
						return m_driverController.getRightX() / OiConstants.slowTurningDivisor * -1;
					}
					return m_driverController.getRightX() * -1;
				}
				return 0.0;
			},
			() -> !m_driverController.getHID().getRightBumper());

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the trigger bindings
		configureBindings();

		m_swerveDrive.setDefaultCommand(m_SwerveDriveCommand);
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be
	 * created via the
	 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
	 * an arbitrary
	 * predicate, or via the named factories in {@link
	 * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
	 * {@link
	 * CommandXboxController
	 * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
	 * PS4} controllers or
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
	 * joysticks}.
	 */
	private void configureBindings() {
		// Schedule `ExampleCommand` when `exampleCondition` changes to `true`
		new Trigger(m_exampleSubsystem::exampleCondition)
				.onTrue(new ExampleCommand(m_exampleSubsystem));

		// Schedule `exampleMethodCommand` when the Xbox controller's B button is
		// pressed,
		// cancelling on release.
		m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return Autos.exampleAuto(m_exampleSubsystem);
	}
}
