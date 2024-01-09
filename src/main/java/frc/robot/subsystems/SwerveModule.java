package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.SwerveConstants;

/** An individual swerve module. */
public class SwerveModule {
    // Drive
    private final TalonFX driveController;

    // Turning
    private final CANSparkMax turningController;
    private final PIDController turningPid;

    // CAN Coder
    private final CANcoder canCoder;
    private final double canCoderOffset;

    /**
     * Creates a new swerve module.
     *
     * @param driveId CAN ID for the drive motor (Falcon)
     * @param isDriveReversed If the drive motor is reversed
     * @param turningId CAN ID for the turning motor (NEO)
     * @param isTurningReversed If the turning motor is reversed
     * @param coderId CAN ID for the CAN coder within the swerve module
     * @param coderOffset The offset to get the CAN coder to true zero (NEED TO BE POSITIVE)
     */
    public SwerveModule(
            int driveId,
            boolean isDriveReversed,
            int turningId,
            boolean isTurningReversed,
            int coderId,
            double coderOffset) {
        driveController = new TalonFX(driveId);
        driveController.setInverted(isDriveReversed);

        turningController = new CANSparkMax(turningId, MotorType.kBrushless);
        turningController.setInverted(isTurningReversed);
        turningController.restoreFactoryDefaults(true);
        turningController.setIdleMode(IdleMode.kBrake);

        turningPid =
                new PIDController(
                        SwerveConstants.PIDConstants.kP,
                        SwerveConstants.PIDConstants.kI,
                        SwerveConstants.PIDConstants.kD);
        turningPid.enableContinuousInput(0.0, 1.0);
        turningPid.setTolerance(SwerveConstants.PIDConstants.kT);

        canCoder = new CANcoder(coderId);

        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        canCoder.getConfigurator().apply(config);

        this.canCoderOffset = coderOffset;
    }

    /**
     * Sets the swerve module to a new state.
     *
     * @param state The new desired state
     * @param log If the Swerve Module should log there target state and current state
     */
    public void setState(SwerveModuleState state, boolean log) {
        if (log) {
            SmartDashboard.putString(
                    "Swerve "
                            + Integer.toString(driveController.getDeviceID() - 10).charAt(0)
                            + " Target State",
                    String.format(
                            "Speed: %.3f, Rotation: %.3f",
                            state.speedMetersPerSecond, state.angle.getRadians()));
            SmartDashboard.putNumber(
                    "Swerve "
                            + Integer.toString(driveController.getDeviceID() - 10).charAt(0)
                            + " Current Rotation",
                    getTurningPosition());
        }

        // Implements a "deadzone" so releasing the joystick won't make the wheels reset to 0
        if (Math.abs(state.speedMetersPerSecond) <= .01) {
            stop();
            return;
        }

        // Optimize movements to not move more than 90 deg for any new state
        state = SwerveModuleState.optimize(state, Rotation2d.fromRadians(getTurningPosition()));
        driveController.set(
                state.speedMetersPerSecond / SwerveConstants.Dimensions.drivePhysicalMaxSpeed);
        turningController.set(
                turningPid.calculate(
                        getTurningPosition(), state.angle.getRadians() / (2 * Math.PI)));
    }

    /** Stops the swerve module. */
    public void stop() {
        driveController.set(0);
        turningController.set(0);
    }

    /**
     * Gets the turning motor position in radians.
     *
     * @return Turning motor position
     */
    public double getTurningPosition() {
        return canCoder.getPosition().getValueAsDouble() - canCoderOffset;
    }

    /**
     * Gets the turning motor position in radians in a human readable form.
     *
     * @return Turning motor position
     */
    public double getTurningPositionReadable() {
        return Math.abs((canCoder.getPosition().getValueAsDouble() - canCoderOffset) % 1)
                * 2
                * Math.PI;
    }
}
