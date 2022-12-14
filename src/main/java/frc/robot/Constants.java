package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.util.Copied;
import frc.robot.util.MathHelper;

public final class Constants {
    @Copied
    public static class SwerveModule {
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
        public static final double DRIVE_GEAR_RATIO = 1 / 8.14;
        public static final double SWERVE_GEAR_RATIO = 7.0 / 150.0;
        public static final double DRIVE_ROT_2 = DRIVE_GEAR_RATIO * Math.PI * WHEEL_DIAMETER; // m
        public static final double SWERVE_ROT_2 = MathHelper.roundToRadian(SWERVE_GEAR_RATIO); // Rad
        public static final double DRIVE_RPM_2 = DRIVE_ROT_2 / 60; // m/s
        public static final double SWERVE_RPM_2 = SWERVE_ROT_2 / 60; // Rad/s
        public static final double P_SWERVE = 0.5;
    }

    public static class Elevator {
        public static final double CW_POS_FAC = 1.0;
        public static final double CW_VEL_FAC = 1.0;
        public static final double CCW_POS_FAC = 1.0;
        public static final double CCW_VEL_FAC = 1.0;
        public static final double MAX_SPD_RATIO = 0.5;
        public static final double MAX_PERMITTED_ROUND = 5;
        public static final double BUFFER_ROUND = 0.1;
    }

    @Copied
    public static class Drive {
        public static final double R_L_DISTANCE = Units.inchesToMeters(25);
        public static final double F_B_DISTANCE = Units.inchesToMeters(25);
        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(F_B_DISTANCE / 2, -R_L_DISTANCE / 2), new Translation2d(F_B_DISTANCE / 2, R_L_DISTANCE / 2),
                new Translation2d(-F_B_DISTANCE / 2, -R_L_DISTANCE / 2), new Translation2d(-F_B_DISTANCE / 2, R_L_DISTANCE / 2)
        );

        public static final boolean FL_SWERVE_INVERTED = true;
        public static final boolean BL_SWERVE_INVERTED = true;
        public static final boolean FR_SWERVE_INVERTED = true;
        public static final boolean BR_SWERVE_INVERTED = true;
        public static final boolean FL_DRIVE_INVERTED = true;
        public static final boolean BL_DRIVE_INVERTED = true;
        public static final boolean FR_DRIVE_INVERTED = false;
        public static final boolean BR_DRIVE_INVERTED = false;
        public static final boolean FL_ABS_INVERTED = false;
        public static final boolean BL_ABS_INVERTED = false;
        public static final boolean FR_ABS_INVERTED = false;
        public static final boolean BR_ABS_INVERTED = false;
        public static final double FL_ABS_OFFSET = 297.5; // Rad
        public static final double BL_ABS_OFFSET = 90.08; // Rad
        public static final double FR_ABS_OFFSET = 290.39; // Rad
        public static final double BR_ABS_OFFSET = 31.2; // Rad
        public static final double MAX_DRIVE_SPD$PHYSICAL = 5; // m/s
        public static final double MAX_SWERVE_SPD$PHYSICAL = MathHelper.roundToRadian(2.0);

        public static final double MAX_DRIVE_SPD = MAX_DRIVE_SPD$PHYSICAL / 4; // m/s
        public static final double MAX_SWERVE_SPD = MAX_SWERVE_SPD$PHYSICAL / 4;
        public static final double MAX_DRIVE_ACC = 3; // m/s^2
        public static final double MAX_SWERVE_ACC = 3; // Rad/s^2
    }

    @Copied
    public static class JoyStick {
        public static final int CONTROL_PORT = 0;
        public static final int X_AXIS = 0;
        public static final int Y_AXIS = 1;
        public static final int SWERVE_AXIS = 2;
        public static final int FIELD_ORIENTED_IDX = 1;
        public static final int ELEVATOR_RISE_IDX = 2;
        public static final int ELEVATOR_LAND_IDX = 3;
        public static final double DEADBAND = 0.05;
    }
}
