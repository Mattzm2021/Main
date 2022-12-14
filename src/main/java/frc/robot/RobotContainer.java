package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.JoystickCommand;
import frc.robot.subsystems.ElevatorSystem;
import frc.robot.subsystems.SwerveSystem;

public class RobotContainer {
    private final SwerveSystem swerve = new SwerveSystem();
    private final ElevatorSystem elevatorSystem = new ElevatorSystem(RobotMap.ElevatorPort.CW, RobotMap.ElevatorPort.CCW);
    private final Joystick joystick = new Joystick(Constants.JoyStick.CONTROL_PORT);

    public RobotContainer() {
        this.swerve.setDefaultCommand(new JoystickCommand(this.swerve,
                () -> -this.joystick.getRawAxis(Constants.JoyStick.Y_AXIS), () -> this.joystick.getRawAxis(Constants.JoyStick.X_AXIS),
                () -> this.joystick.getRawAxis(Constants.JoyStick.SWERVE_AXIS), () -> !this.joystick.getRawButton(Constants.JoyStick.FIELD_ORIENTED_IDX)));
        this.elevatorSystem.setDefaultCommand(new ElevatorCommand(this.elevatorSystem, () -> this.joystick.getRawButton(Constants.JoyStick.ELEVATOR_RISE_IDX),
                () -> this.joystick.getRawButton(Constants.JoyStick.ELEVATOR_RISE_IDX) != this.joystick.getRawButton(Constants.JoyStick.ELEVATOR_LAND_IDX)));
        this.configureButtonBindings();
    }

    private void configureButtonBindings() {

    }
}
