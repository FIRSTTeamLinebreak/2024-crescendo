package frc.robot;

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
        public static final double wheelDiameter = Units.inchesToMeters(2);
        public static final double driveGearRatio = 6.12;

        public static final double driveRotToMeters =
                ((2 * Math.PI * wheelDiameter) / driveGearRatio) / 2048;
        public static final double driveRpsToMps = driveRotToMeters / 10 * 60 * 2;

        public static final double drivePhysicalMaxSpeed = 5.486;

        public static final double trackWidth = Units.inchesToMeters(23.5);
        public static final double wheelBase = Units.inchesToMeters(21.5);

        public static final Translation2d locationFL =
                new Translation2d(wheelBase / 2, trackWidth / 2);
        public static final Translation2d locationFR =
                new Translation2d(wheelBase / 2, -trackWidth / 2);
        public static final Translation2d locationBL =
                new Translation2d(-wheelBase / 2, trackWidth / 2);
        public static final Translation2d locationBR =
                new Translation2d(-wheelBase / 2, -trackWidth / 2);

        public static final double offsetFL = 0.219;
        public static final double offsetFR = 0.876;
        public static final double offsetBL = 0.397;
        public static final double offsetBR = 0.720;

        // frontLeft = new SwerveModule(21, false, 22, false, 23, 4.516 - Math.PI);
        // frontRight = new SwerveModule(31, false, 32, false, 33, 5.604);
        // backLeft = new SwerveModule(41, false, 42, false, 43, 5.647 - Math.PI);
        // backRight = new SwerveModule(51, false, 52, false, 53, 4.522);
        public static final SwerveDriveKinematics driveKinematics =
                new SwerveDriveKinematics(locationFL, locationFR, locationBL, locationBR);
    }

    /** Constants pertaining to the joystick. */
    public static final class JoystickConstants {
        public static final int driveControllerId = 0;
        public static final int scoreControllerId = 1;

        public static final double joystickDeadZone = 0.1;
        public static final double triggerDeadZone = 0.1;

        public static final double xySpeedMultiplier = 1; // Speed multiplier in m/s(?)
        public static final double turningSpeedMultiplier = 1.25;
        public static final double slowTurningDivisor = 2;
    }
}
