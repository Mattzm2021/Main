package frc.robot;

import frc.robot.util.Copied;

public class RobotMap {
    @Copied
    public static class DriverPort {
        public static final int FL_DRIVE = 2;
        public static final int BL_DRIVE = 5;
        public static final int FR_DRIVE = 4;
        public static final int BR_DRIVE = 7;
        public static final int FL_SWERVE = 1;
        public static final int BL_SWERVE = 6;
        public static final int FR_SWERVE = 3;
        public static final int BR_SWERVE = 8;
        public static final int FL_ABS = 9;
        public static final int BL_ABS = 11;
        public static final int FR_ABS = 10;
        public static final int BR_ABS = 12;
    }

    public static class ElevatorPort {
        public static final int CW = 13;
        public static final int CCW = 14;
    }
}
