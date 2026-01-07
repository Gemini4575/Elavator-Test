package frc.robot.commands.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ManulElevaorCommand extends Command {
    private final ElevatorSubsystem m_ElevatorSubsystem;
    private final DoubleSupplier joystickSupplier;

    public ManulElevaorCommand(DoubleSupplier joystickSupplier, ElevatorSubsystem elevatorSubsystem) {
        this.m_ElevatorSubsystem = elevatorSubsystem;
        this.joystickSupplier = joystickSupplier;
    }

    @Override
    public void execute() {
        m_ElevatorSubsystem.moveWithJoysticks(joystickSupplier.getAsDouble());
    }
}
