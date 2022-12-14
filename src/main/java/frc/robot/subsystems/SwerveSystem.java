package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.util.Copied;

/**
 * this is the main swerve subsystem base that includes 4 modules
 */
@Copied
public class SwerveSystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            RobotMap.DriverPort.FL_DRIVE, RobotMap.DriverPort.FL_SWERVE,
            Constants.Drive.FL_DRIVE_INVERTED, Constants.Drive.FL_SWERVE_INVERTED,
            RobotMap.DriverPort.FL_ABS, Constants.Drive.FL_ABS_INVERTED, Constants.Drive.FL_ABS_OFFSET
    );

    private final SwerveModule frontRight = new SwerveModule(
            RobotMap.DriverPort.FR_DRIVE, RobotMap.DriverPort.FR_SWERVE,
            Constants.Drive.FR_DRIVE_INVERTED, Constants.Drive.FR_SWERVE_INVERTED,
            RobotMap.DriverPort.FR_ABS, Constants.Drive.FR_ABS_INVERTED, Constants.Drive.FR_ABS_OFFSET
    );

    private final SwerveModule backLeft = new SwerveModule(
            RobotMap.DriverPort.BL_DRIVE, RobotMap.DriverPort.BL_SWERVE,
            Constants.Drive.BL_DRIVE_INVERTED, Constants.Drive.BL_SWERVE_INVERTED,
            RobotMap.DriverPort.BL_ABS, Constants.Drive.BL_ABS_INVERTED, Constants.Drive.BL_ABS_OFFSET
    );

    private final SwerveModule backRight = new SwerveModule(
            RobotMap.DriverPort.BR_DRIVE, RobotMap.DriverPort.BR_SWERVE,
            Constants.Drive.BR_DRIVE_INVERTED, Constants.Drive.BR_SWERVE_INVERTED,
            RobotMap.DriverPort.BR_ABS, Constants.Drive.BR_ABS_INVERTED, Constants.Drive.BR_ABS_OFFSET
    );

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(Constants.Drive.DRIVE_KINEMATICS, new Rotation2d(0.0));

    public SwerveSystem() {
        // to delaying the initialization of the gyro due to some scheduling limits
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                this.zeroHeading();
            } catch (Exception ignored) {}
        }).start();
    }

    @Override
    public void periodic() {
        odometer.update(this.getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState());
        SmartDashboard.putNumber("Robot Heading", this.getHeading());
        SmartDashboard.putString("Robot Location", this.getPose2d().getTranslation().toString());
        this.putModuleDashboard();
    }

    public double getHeading() {
        return Math.IEEEremainder(-gyro.getAngle(), 360.0);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(this.getHeading());
    }

    public Pose2d getPose2d() {
        return this.odometer.getPoseMeters();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public void resetOdometer(Pose2d pose2d) {
        odometer.resetPosition(pose2d, this.getRotation2d());
    }

    public void stopModules() {
        this.frontLeft.stop();
        this.frontRight.stop();
        this.backLeft.stop();
        this.backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Drive.MAX_DRIVE_SPD$PHYSICAL);
        this.frontLeft.setDesiredState(states[0]);
        this.frontRight.setDesiredState(states[1]);
        this.backLeft.setDesiredState(states[2]);
        this.backRight.setDesiredState(states[3]);
    }

    public void resetSwerveMotors() {
        this.frontLeft.resetSwerveMotor();
        this.frontRight.resetSwerveMotor();
        this.backLeft.resetSwerveMotor();
        this.backRight.resetSwerveMotor();
    }

    private void putModuleDashboard() {
        this.frontLeft.putDashboard();
        this.frontRight.putDashboard();
        this.backLeft.putDashboard();
        this.backRight.putDashboard();
    }
}
