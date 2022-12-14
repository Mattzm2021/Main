package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSystem;
import frc.robot.util.Copied;
import org.jetbrains.annotations.NotNull;

import java.util.function.Supplier;

/**
 * this is the command that handles the drive input from joystick. Notice that this does not include elevator operation.
 */
@Copied
public class JoystickCommand extends CommandBase {
    private final SwerveSystem swerve;
    private final Supplier<Double> xSpdFunc, ySpdFunc, swerveSpdFunc; // determine the anticipated absolute speeds
    private final Supplier<Boolean> fieldOrientedFunc; // determine whether the operation is field oriented
    private final SlewRateLimiter xLimiter, yLimiter, swerveLimiter; // determine the speed limiters

    public JoystickCommand(SwerveSystem swerve, Supplier<Double> xSpdFunc, Supplier<Double> ySpdFunc, Supplier<Double> swerveSpdFunc,
                           Supplier<Boolean> fieldOrientedFunc) {
        this.swerve = swerve;
        this.xSpdFunc = xSpdFunc;
        this.ySpdFunc = ySpdFunc;
        this.swerveSpdFunc = swerveSpdFunc;
        this.fieldOrientedFunc = fieldOrientedFunc;
        this.xLimiter = new SlewRateLimiter(Constants.Drive.MAX_DRIVE_ACC);
        this.yLimiter = new SlewRateLimiter(Constants.Drive.MAX_DRIVE_ACC);
        this.swerveLimiter = new SlewRateLimiter(Constants.Drive.MAX_SWERVE_ACC);
        this.addRequirements(swerve);
    }

    @Override
    public void execute() {
        this.considerDrive();
    }

    @Override
    public void end(boolean interrupted) {
        this.swerve.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // consider drive operation
    private void considerDrive() {
        double xSpd = applyDeadband(this.xSpdFunc.get()), ySpd = applyDeadband(this.ySpdFunc.get()), swerveSpd = applyDeadband(this.swerveSpdFunc.get());
        // if the operation is ignorable, reset motor and return
        if (xSpd == 0 && ySpd == 0 && swerveSpd == 0) {
            this.swerve.resetSwerveMotors();
            return;
        }

        xSpd = applyLimiter(xSpd, this.xLimiter, Constants.Drive.MAX_DRIVE_SPD);
        ySpd = applyLimiter(ySpd, this.yLimiter, Constants.Drive.MAX_DRIVE_SPD);
        swerveSpd = applyLimiter(swerveSpd, this.swerveLimiter, Constants.Drive.MAX_SWERVE_SPD);
        ChassisSpeeds speeds = this.getCurrentChassis(xSpd, ySpd, swerveSpd, this.fieldOrientedFunc.get());
        this.setModuleStates(speeds);
    }

    private static double applyDeadband(double d) {
        return Math.abs(d) > Constants.JoyStick.DEADBAND ? d : 0.0;
    }

    private static double applyLimiter(double d, @NotNull SlewRateLimiter limiter, double maxSpeed) {
        return limiter.calculate(d) * maxSpeed;
    }

    private @NotNull ChassisSpeeds getCurrentChassis(double xSpd, double ySpd, double swerveSpd, boolean isFieldOriented) {
        if (isFieldOriented) {
            return ChassisSpeeds.fromFieldRelativeSpeeds(xSpd, ySpd, swerveSpd, this.swerve.getRotation2d());
        } else {
            return new ChassisSpeeds(xSpd, ySpd, swerveSpd);
        }
    }

    private void setModuleStates(ChassisSpeeds speeds) {
        this.swerve.setModuleStates(Constants.Drive.DRIVE_KINEMATICS.toSwerveModuleStates(speeds));
    }
}
