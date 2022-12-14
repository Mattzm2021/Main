package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.MotorHelper;


/**
 * this is the elevator subsystem that includes 2 motors
 */
public class ElevatorSystem extends SubsystemBase {
    private final CANSparkMax cwMotor;
    private final CANSparkMax ccwMotor;

    public ElevatorSystem(int cwId, int ccwId) {
        this.cwMotor = new CANSparkMax(cwId, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.ccwMotor = new CANSparkMax(ccwId, CANSparkMaxLowLevel.MotorType.kBrushless);
        MotorHelper.initMotor(this.cwMotor, Constants.Elevator.CW_POS_FAC, Constants.Elevator.CW_VEL_FAC);
        MotorHelper.initMotor(this.ccwMotor, Constants.Elevator.CCW_POS_FAC, Constants.Elevator.CCW_VEL_FAC);
    }

    public double getMotorPosition() {
        return this.cwMotor.getEncoder().getPosition();
    }

    // represents whether the elevator is inside its freely moving range
    public boolean isFlexible() {
        return !(this.getMotorPosition() > Constants.Elevator.MAX_PERMITTED_ROUND - Constants.Elevator.BUFFER_ROUND) && !(this.getMotorPosition() < Constants.Elevator.BUFFER_ROUND);
    }

    public void rise() {
        this.cwMotor.set(Constants.Elevator.MAX_SPD_RATIO);
        this.ccwMotor.set(Constants.Elevator.MAX_SPD_RATIO);
    }

    public void land() {
        this.cwMotor.set(-Constants.Elevator.MAX_SPD_RATIO);
        this.ccwMotor.set(-Constants.Elevator.MAX_SPD_RATIO);
    }

    public void stop() {
        this.cwMotor.stopMotor();
        this.ccwMotor.stopMotor();
    }

    // to automatically locate the elevator to the y = 0 bottom
    public void reCoordinate() {
        if (this.isFlexible()) {
            this.land();
        } else {
            this.stop();
        }
    }
}
