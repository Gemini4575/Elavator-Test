// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.JoystickConstants;
import frc.robot.commands.elevator.ManulElevaorCommand;
import frc.robot.subsystems.ElevatorSubsystem;

import java.util.Map;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RobotContainer {
  ShuffleboardLayout elevatorCommands = Shuffleboard.getTab("Eleavator")
      .getLayout("Elevator Commands", BuiltInLayouts.kList)
      .withSize(2, 2)
      .withProperties(Map.of("Label position", "LEFT")); // hide labels for commands
  // The robot's subsystems and commands are defined here...
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController m_driverController = new XboxController(JoystickConstants.DRIVER_USB);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    elevatorCommands.add("Flash Configs", new InstantCommand(() -> m_ElevatorSubsystem.FlashConfigs()));
    elevatorCommands.add("Move to middle", m_ElevatorSubsystem.moveToHeightCommand(0.5));
    elevatorCommands.add("Move to 0", m_ElevatorSubsystem.moveToHeightCommand(0.0));
    m_ElevatorSubsystem
        .setDefaultCommand(new ManulElevaorCommand(() -> m_driverController.getLeftX(), m_ElevatorSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new InstantCommand(() -> new WaitCommand(2));
  }
}
