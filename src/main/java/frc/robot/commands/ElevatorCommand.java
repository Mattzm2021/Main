package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSystem;

import java.util.function.Supplier;

/**
 * this is the command that handles the operation of elevator provided by the joystick.
 */
public class ElevatorCommand extends CommandBase {
    private final ElevatorSystem elevatorSystem;
    private final Supplier<Boolean> directionFunc, activeFunc;

    public ElevatorCommand(ElevatorSystem elevatorSystem, Supplier<Boolean> directionFunc, Supplier<Boolean> activeFunc) {
        this.elevatorSystem = elevatorSystem;
        this.directionFunc = directionFunc;
        this.activeFunc = activeFunc;
        this.addRequirements(elevatorSystem);
    }

    @Override
    public void execute() {
        // make sure the elevator is operated and does not transgress the limitation
        if (this.activeFunc.get() && this.elevatorSystem.isFlexible()) {
            if (this.directionFunc.get()) {
                this.elevatorSystem.rise();
            } else {
                this.elevatorSystem.land();
            }
        } else {
            this.elevatorSystem.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.elevatorSystem.reCoordinate();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
