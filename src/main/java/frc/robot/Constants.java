// Copyright (c) FIRST and other WPILib contributors.
// the WPILib BSD license file in the root directory of this project.
// Open Source Software; you can modify and/or share it under the terms of

package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public final class Constants {
  public final static class JoystickConstants {
    public final static int DRIVER_USB = 0;
    public final static int OPERATOR_USB = 1;
    public final static int TEST_USB = 2;

    public final static int LEFT_Y_AXIS = 1;
    public final static int LEFT_X_AXIS = 0;
    public final static int RIGHT_X_AXIS = 4;
    public final static int RIGHT_Y_AXIS = 5;

    public final static int GREEN_BUTTON = 1;
    public final static int RED_BUTTON = 2;
    public final static int YELLOW_BUTTON = 4;
    public final static int BLUE_BUTTON = 3;

    public final static int LEFT_TRIGGER = 2;
    public final static int RIGHT_TRIGGER = 3;
    public final static int LEFT_BUMPER = 5;
    public final static int RIGHT_BUMPER = 6;

    public final static int BACK_BUTTON = 7;
    public final static int START_BUTTON = 8;

    public final static int POV_UP = 0;
    public final static int POV_RIGHT = 90;
    public final static int POV_DOWN = 180;
    public final static int POV_LEFT = 270;
  }

  public static class ElevatorConstants {
    public static final ShuffleboardTab ELEVATOR_SHUFFLEBOARD_TAB = Shuffleboard.getTab("Eleavator");
    public static final GenericEntry elevatorPosition_entery = ELEVATOR_SHUFFLEBOARD_TAB
        .add("Elevator Position", 0)
        .withPosition(0, 0)
        .withSize(2, 1)
        .getEntry();
    public static final GenericEntry elevatorTargetPosition_entery = ELEVATOR_SHUFFLEBOARD_TAB
        .add("Elevator Target Position", 0)
        .withPosition(2, 0)
        .withSize(2, 1)
        .getEntry();
    public static final GenericEntry kP_entry = ELEVATOR_SHUFFLEBOARD_TAB
        .add("Elevator kP", 0)
        .withPosition(0, 1)
        .withSize(1, 1)
        .getEntry();
    public static final GenericEntry kI_entry = ELEVATOR_SHUFFLEBOARD_TAB
        .add("Elevator kI", 0)
        .withPosition(1, 1)
        .withSize(1, 1)
        .getEntry();
    public static final GenericEntry kD_entry = ELEVATOR_SHUFFLEBOARD_TAB
        .add("Elevator kD", 0)
        .withPosition(2, 1)
        .withSize(1, 1)
        .getEntry();
    public static final ShuffleboardLayout ELEVATOR_PID_LAYOUT = ELEVATOR_SHUFFLEBOARD_TAB
        .getLayout("Elevator PID", "Grid")
        .withPosition(0, 2)
        .withSize(3, 2);

    // Constants
    public final static int leaderCanID = 1;
    public final static int followerCanID = 2;
    public final static double gearRatio = 4.45;
    // for right now this is blank because we will tune it from shuffleboard
    /*
     * PID Constants - tuned for Kraken X60
     * public final static double kP = 50.0; // Higher P for position control
     * public final static double kI = 0.0;
     * public final static double kD = 2.0; // Some D for damping
     */

    // Feedforward Constants - typical for Kraken X60
    public final static double kS = 0.25; // Volts to overcome static friction
    public final static double kG = 0.35; // Volts to hold position against gravity
    public final static double kV = 12.0 / 6000.0 * 60.0; // Volts per (rot/s) - based on free speed ~6000 RPM
    public final static double kA = 0.01; // Volts per (rot/s^2)

    // Motion constraints
    public final static double maxVelocity = 1.5; // meters per second
    public final static double maxAcceleration = 3.0; // meters per second squared

    // Hardware settings
    public final static boolean brakeMode = true;
    public final static boolean enableStatorLimit = true;
    public final static int statorCurrentLimit = 60; // Amps - Kraken can handle more
    public final static boolean enableSupplyLimit = true;
    public final static double supplyCurrentLimit = 50; // Amps

    // Mechanical constants
    public final static double drumRadius = Units.inchesToMeters(0.675); // 1 inch drum radius
    public final static double minHeight = 0.0; // meters
    public final static double maxHeight = 1.2; // meters
    public final static double carriageMass = Units.lbsToKilograms(3.323); // kg - adjust based on your carriage

  }
}
