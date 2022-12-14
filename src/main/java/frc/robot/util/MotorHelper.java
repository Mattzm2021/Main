package frc.robot.util;

import com.revrobotics.CANSparkMax;
import org.jetbrains.annotations.NotNull;

public class MotorHelper {
    public static void initMotor(@NotNull CANSparkMax motor, double posFactor, double velFactor) {
        initMotor(motor, false, false, posFactor, velFactor);
    }

    public static void initMotor(@NotNull CANSparkMax motor, boolean isCoast, boolean inverted, double posFactor, double velFactor) {
        motor.setIdleMode(isCoast ? CANSparkMax.IdleMode.kCoast : CANSparkMax.IdleMode.kBrake);
        motor.setInverted(inverted);
        motor.getEncoder().setPositionConversionFactor(posFactor);
        motor.getEncoder().setVelocityConversionFactor(velFactor);
    }
}
