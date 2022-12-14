package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Copied;
import frc.robot.util.MathHelper;
import frc.robot.util.MotorHelper;
import org.jetbrains.annotations.NotNull;

/**
 * this is a single module of swerve drive. We should have 4 of this module to construct a subsystem
 */
@Copied
public class SwerveModule extends SubsystemBase {
    private final CANSparkMax driveMotor;
    private final CANSparkMax swerveMotor;
    private final PIDController swerveController;
    private final CANCoder absEncoder; // track the absolute position of the robot currently
    private final boolean absEncoderInverted;
    private final double absEncoderOffset;

    public SwerveModule(int driveId, int swerveId, boolean driveInverted, boolean swerveInverted, int absEncoderId, boolean absEncoderInverted, double absEncoderOffset) {
        this.driveMotor = new CANSparkMax(driveId, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.swerveMotor = new CANSparkMax(swerveId, CANSparkMaxLowLevel.MotorType.kBrushless);
        MotorHelper.initMotor(this.driveMotor, true, driveInverted, Constants.SwerveModule.DRIVE_ROT_2, Constants.SwerveModule.DRIVE_RPM_2);
        MotorHelper.initMotor(this.swerveMotor, true, swerveInverted, Constants.SwerveModule.SWERVE_ROT_2, Constants.SwerveModule.SWERVE_RPM_2);
        this.swerveController = new PIDController(Constants.SwerveModule.P_SWERVE, 0.0, 0.0);
        this.swerveController.enableContinuousInput(-Math.PI, Math.PI); // since -PI to PI is a round, active continuous intervals
        this.absEncoder = new CANCoder(absEncoderId);
        this.absEncoder.setPositionToAbsolute();
        this.absEncoderInverted = absEncoderInverted;
        this.absEncoderOffset = absEncoderOffset;
        this.resetEncoders();
        this.putDashboard(); // output data to viewer
    }

    public double getDrivePosition() {
        return this.driveMotor.getEncoder().getPosition();
    }

    public double getSwervePosition() {
        return this.swerveMotor.getEncoder().getPosition();
    }

    public double getDriveVelocity() {
        return this.driveMotor.getEncoder().getVelocity();
    }

    public double getSwerveVelocity() {
        return this.swerveMotor.getEncoder().getVelocity();
    }

    public double getAbsEncoderRad() {
        double angle = (this.absEncoder.getAbsolutePosition() - this.absEncoderOffset) / 360.0;
        return MathHelper.roundToRadian(angle) * this.getInverseCoefficient();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(this.getDriveVelocity(), new Rotation2d(this.getSwervePosition()));
    }

    public void setDesiredState(@NotNull SwerveModuleState state) {
        // ignore the ignorable input state
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            this.stop();
            return;
        }

        state = SwerveModuleState.optimize(state, this.getState().angle);
        this.driveMotor.set(state.speedMetersPerSecond / Constants.Drive.MAX_DRIVE_SPD$PHYSICAL);
        this.swerveMotor.set(this.swerveController.calculate(this.getSwervePosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + this.absEncoder.getDeviceID() + "] state", state.toString());
        this.putDashboard();
    }

    public void stop() {
        this.driveMotor.set(0.0);
        this.swerveMotor.set(0.0);
    }

    public void resetEncoders() {
        this.driveMotor.getEncoder().setPosition(0);
        this.swerveMotor.getEncoder().setPosition(this.getAbsEncoderRad());
    }

    public void resetSwerveMotor() {
        if (Math.abs(this.swerveMotor.getEncoder().getPosition()) < 0.1) {
            this.stop();
            return;
        }

        this.swerveMotor.set(this.swerveController.calculate(this.swerveMotor.getEncoder().getPosition(), 0));
        this.driveMotor.set(0.0);
        this.putDashboard();
    }

    public void putDashboard() {
        SmartDashboard.putNumber("ABS angle " + this.absEncoder.getDeviceID(), this.getAbsEncoderRad());
        SmartDashboard.putNumber("Abs Position " + this.absEncoder.getDeviceID(), this.absEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Position " + this.absEncoder.getDeviceID(), this.absEncoder.getPosition());
        SmartDashboard.putNumber("Turing position " + this.swerveMotor.getDeviceId(), this.swerveMotor.getEncoder().getPosition());
    }

    private int getInverseCoefficient() {
        return this.absEncoderInverted ? -1 : 1;
    }
}
