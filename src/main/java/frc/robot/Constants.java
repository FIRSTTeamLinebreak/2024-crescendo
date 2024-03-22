package frc.robot;

import static java.lang.Math.PI;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /** Contstants pertaining to the Swerve Modules */
    public static final class SwerveConstants {
        /** Dimensions for the Swerve Modules */
        public static final class Dimensions {
            public static final double wheelDiameter = Units.inchesToMeters(4.0);
            public static final double wheelCircumference = wheelDiameter * PI;

            public static final double wheelBase = Units.inchesToMeters(21.5);
            public static final double trackWidth = Units.inchesToMeters(23.5);

            public static final double magOffsetFL = 0.149;
            public static final double magOffsetFR = 0.966;
            public static final double magOffsetBL = 0.466;
            public static final double magOffsetBR = 0.642;
        }

        public static final class Kinematics {
            public static final Translation2d locationFL =
                    new Translation2d(Dimensions.wheelBase / 2, Dimensions.trackWidth / 2);
            public static final Translation2d locationFR =
                    new Translation2d(Dimensions.wheelBase / 2, -Dimensions.trackWidth / 2);
            public static final Translation2d locationBL =
                    new Translation2d(-Dimensions.wheelBase / 2, Dimensions.trackWidth / 2);
            public static final Translation2d locationBR =
                    new Translation2d(-Dimensions.wheelBase / 2, -Dimensions.trackWidth / 2);

            public static final SwerveDriveKinematics driveKinematics =
                    new SwerveDriveKinematics(locationFL, locationFR, locationBL, locationBR);

            public static final double driveGearRatio = 6.12;
            public static final double driveRotToMeters =
                    Dimensions.wheelCircumference / driveGearRatio;
            public static final double driveRpsToMps = driveRotToMeters / 10 * 60 * 2;

            public static final double drivePhysicalMaxSpeed = 5.486;
        }

        public static final class PID {
            public static final class ModuleAngle {
                public static final double kP = 1.5;
                public static final double kI = 0.0;
                public static final double kD = 0.005;

                public static final double kT = 0.0005;
            }

            public static final class ClawRotation {
                public static final double kP = 0.75;
                public static final double kI = 0.0;
                public static final double kD = 0.003;
                public static final double kFF = 0.0275;

                public static final double kT = 0.005;
            }

            public static final class AutoTranslation {
                public static final double kP = 5.0;
                public static final double kI = 0.0;
                public static final double kD = 0.0;

                public static final double kT = 0.01;
            }

            public static final class AutoRotation {
                public static final double kP = 1.0;
                public static final double kI = 0.0;
                public static final double kD = 0.0;

                public static final double kT = 0.01;
            }

            public static final class Elevator {
                public static final double kP = 0.15;
                public static final double kI = 0.0;
                public static final double kD = 0.0;

                public static final double kT = 1.0;
            }
        }

        public static final HolonomicPathFollowerConfig HolonomicConfig =
            new HolonomicPathFollowerConfig(
                    new PIDConstants(
                            PID.AutoTranslation.kP,
                            PID.AutoTranslation.kI,
                            PID.AutoTranslation.kD),
                    new PIDConstants(
                            PID.AutoRotation.kP,
                            PID.AutoRotation.kI,
                            PID.AutoRotation.kD),
                    Kinematics.drivePhysicalMaxSpeed,
                    Math.sqrt(
                        Math.pow(Dimensions.wheelBase/2, 2)
                        + Math.pow(Dimensions.trackWidth/2, 2)),
                    new ReplanningConfig());
    }

    /** Constants pertaining to the joystick. */
    public static final class JoystickConstants {
        public static final int driveControllerId = 0;
        public static final int scoreControllerId = 1;

        public static final double joystickDeadZone = 0.1;
        public static final double triggerDeadZone = 0.1;

        public static final double xySpeedMultiplier = 4.5; // m/s
        public static final double turningSpeedMultiplier = 3.5; // rads/s
        public static final double slowTurningDivisor = 2;
    }
}
