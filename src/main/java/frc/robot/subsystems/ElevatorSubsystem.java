package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Elevator.*;

/**
 * Elevator subsystem using dual TalonFX with Kraken X60 motors
 */
public class ElevatorSubsystem extends SubsystemBase {
  private final DCMotor dcMotor = DCMotor.getKrakenX60(2); // 2 motors

  // Feedforward
  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(
      kS,
      kG,
      kV,
      kA);

  // Motor controllers
  private final TalonFX leader;
  private final TalonFX follower;
  private final PositionVoltage positionRequest;
  private final VelocityVoltage velocityRequest;
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Current> statorCurrentSignal;
  private final StatusSignal<Temperature> temperatureSignal;

  // Simulation
  private final ElevatorSim elevatorSim;

  /**
   * Creates a new Elevator Subsystem.
   */
  public ElevatorSubsystem() {
    // Initialize motor controllers
    leader = new TalonFX(leaderCanID);
    follower = new TalonFX(followerCanID);

    // Create control requests
    positionRequest = new PositionVoltage(0).withSlot(0);
    velocityRequest = new VelocityVoltage(0).withSlot(0);

    // Get status signals from leader
    positionSignal = leader.getPosition();
    velocitySignal = leader.getVelocity();
    voltageSignal = leader.getMotorVoltage();
    statorCurrentSignal = leader.getStatorCurrent();
    temperatureSignal = leader.getDeviceTemp();

    // Configure leader motor
    TalonFXConfiguration leaderConfig = new TalonFXConfiguration();

    // Configure PID for slot 0
    Slot0Configs slot0 = leaderConfig.Slot0;
    slot0.kP = kP;
    slot0.kI = kI;
    slot0.kD = kD;
    slot0.GravityType = GravityTypeValue.Elevator_Static;
    slot0.kS = kS;
    slot0.kV = kV;
    slot0.kA = kA;
    slot0.kG = kG;

    // Set current limits
    CurrentLimitsConfigs currentLimits = leaderConfig.CurrentLimits;
    currentLimits.StatorCurrentLimit = statorCurrentLimit;
    currentLimits.StatorCurrentLimitEnable = enableStatorLimit;
    currentLimits.SupplyCurrentLimit = supplyCurrentLimit;
    currentLimits.SupplyCurrentLimitEnable = enableSupplyLimit;

    // Set brake mode
    leaderConfig.MotorOutput.NeutralMode = brakeMode
        ? NeutralModeValue.Brake
        : NeutralModeValue.Coast;

    // Configure motor inversion (adjust based on your mounting)
    leaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Apply gear ratio
    leaderConfig.Feedback.SensorToMechanismRatio = gearRatio;

    // Configure soft limits
    SoftwareLimitSwitchConfigs softLimits = leaderConfig.SoftwareLimitSwitch;
    softLimits.ForwardSoftLimitEnable = true;
    softLimits.ReverseSoftLimitEnable = true;
    softLimits.ForwardSoftLimitThreshold = metersToRotations(maxHeight);
    softLimits.ReverseSoftLimitThreshold = metersToRotations(minHeight);

    // Configure ramp rates for smooth motion
    OpenLoopRampsConfigs openLoopRamps = leaderConfig.OpenLoopRamps;
    openLoopRamps.VoltageOpenLoopRampPeriod = 0.1; // seconds

    ClosedLoopRampsConfigs closedLoopRamps = leaderConfig.ClosedLoopRamps;
    closedLoopRamps.VoltageClosedLoopRampPeriod = 0.0; // seconds

    // Apply configuration to leader
    leader.getConfigurator().apply(leaderConfig);

    // Configure follower motor
    TalonFXConfiguration followerConfig = new TalonFXConfiguration();

    // Match current limits
    followerConfig.CurrentLimits = currentLimits;

    // Match brake mode
    followerConfig.MotorOutput.NeutralMode = brakeMode
        ? NeutralModeValue.Brake
        : NeutralModeValue.Coast;

    // Invert follower to match leader direction (adjust based on your mounting)
    followerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Apply configuration to follower
    follower.getConfigurator().apply(followerConfig);

    // Set follower to follow leader
    follower.setControl(new Follower(leaderCanID, false));

    // Reset encoder position
    leader.setPosition(0);

    // Initialize simulation (uses both motors)
    elevatorSim = new ElevatorSim(
        dcMotor, // Motor type (2x Kraken X60)
        gearRatio,
        carriageMass, // Carriage mass (kg)
        drumRadius, // Drum radius (m)
        minHeight, // Min height (m)
        maxHeight, // Max height (m)
        true, // Simulate gravity
        0 // Starting height (m)
    );
  }

  /**
   * Update simulation and telemetry.
   */
  @Override
  public void periodic() {
    elevatorPosition_entery.setDouble(getPositionMeters());
    BaseStatusSignal.refreshAll(
        positionSignal,
        velocitySignal,
        voltageSignal,
        statorCurrentSignal,
        temperatureSignal);
  }

  /**
   * Update simulation.
   */
  @Override
  public void simulationPeriodic() {
    // Use motor voltage for TalonFX simulation input
    elevatorSim.setInput(leader.getSimState().getMotorVoltage());

    // Update simulation by 20ms
    elevatorSim.update(0.020);

    // Convert meters to motor rotations
    double motorPosition = metersToRotations(elevatorSim.getPositionMeters());
    double motorVelocity = metersToRotations(elevatorSim.getVelocityMetersPerSecond());

    // Update both motor simulations
    leader.getSimState().setRawRotorPosition(motorPosition);
    leader.getSimState().setRotorVelocity(motorVelocity);
    follower.getSimState().setRawRotorPosition(motorPosition);
    follower.getSimState().setRotorVelocity(motorVelocity);

    // Simulate battery voltage drop
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
  }

  /**
   * Convert meters to motor rotations.
   * 
   * @param meters Distance in meters
   * @return Equivalent rotations
   */
  private double metersToRotations(double meters) {
    return meters / (2.0 * Math.PI * drumRadius) * gearRatio;
  }

  /**
   * Convert rotations to meters.
   * 
   * @param rotations Motor rotations
   * @return Equivalent meters
   */
  private double rotationsToMeters(double rotations) {
    return rotations * (2.0 * Math.PI * drumRadius) / gearRatio;
  }

  /**
   * Get the current position in rotations.
   * 
   * @return Position in rotations
   */
  @Logged(name = "Position/Rotations")
  public double getPositionRotations() {
    return positionSignal.getValueAsDouble();
  }

  /**
   * Get the current position in meters.
   * 
   * @return Position in meters
   */
  @Logged(name = "Position/Meters")
  public double getPositionMeters() {
    return rotationsToMeters(positionSignal.getValueAsDouble());
  }

  /**
   * Get the current velocity in rotations per second.
   * 
   * @return Velocity in rotations per second
   */
  @Logged(name = "Velocity/RotationsPerSecond")
  public double getVelocityRotations() {
    return velocitySignal.getValueAsDouble();
  }

  /**
   * Get the current velocity in meters per second.
   * 
   * @return Velocity in meters per second
   */
  @Logged(name = "Velocity/MetersPerSecond")
  public double getVelocityMeters() {
    return rotationsToMeters(velocitySignal.getValueAsDouble());
  }

  /**
   * Get the current applied voltage.
   * 
   * @return Applied voltage
   */
  @Logged(name = "Voltage")
  public double getVoltage() {
    return voltageSignal.getValueAsDouble();
  }

  /**
   * Get the current motor current.
   * 
   * @return Motor current in amps
   */
  @Logged(name = "Current")
  public double getCurrent() {
    return statorCurrentSignal.getValueAsDouble();
  }

  /**
   * Get the current motor temperature.
   * 
   * @return Motor temperature in Celsius
   */
  @Logged(name = "Temperature")
  public double getTemperature() {
    return temperatureSignal.getValueAsDouble();
  }

  /**
   * Set elevator position.
   * 
   * @param positionMeters The target position in meters
   */
  public void setPosition(double positionMeters) {
    setPosition(positionMeters, 0);
  }

  /**
   * Set elevator position with feedforward acceleration.
   * 
   * @param positionMeters                     The target position in meters
   * @param accelerationMetersPerSecondSquared The acceleration in meters per
   *                                           second squared
   */
  public void setPosition(double positionMeters, double accelerationMetersPerSecondSquared) {
    // Convert meters to rotations
    double positionRotations = metersToRotations(positionMeters);

    // Calculate feedforward
    double ffVolts = feedforward.calculate(getVelocityMeters(), accelerationMetersPerSecondSquared);

    // Send control request with feedforward
    leader.setControl(positionRequest.withPosition(positionRotations).withFeedForward(ffVolts));
  }

  /**
   * Set elevator velocity.
   * 
   * @param velocityMetersPerSecond The target velocity in meters per second
   */
  public void setVelocity(double velocityMetersPerSecond) {
    setVelocity(velocityMetersPerSecond, 0);
  }

  /**
   * Set elevator velocity with feedforward acceleration.
   * 
   * @param velocityMetersPerSecond            The target velocity in meters per
   *                                           second
   * @param accelerationMetersPerSecondSquared The acceleration in meters per
   *                                           second squared
   */
  public void setVelocity(double velocityMetersPerSecond, double accelerationMetersPerSecondSquared) {
    // Convert meters/sec to rotations/sec
    double velocityRotations = metersToRotations(velocityMetersPerSecond);

    // Calculate feedforward
    double ffVolts = feedforward.calculate(velocityMetersPerSecond, accelerationMetersPerSecondSquared);

    // Send control request with feedforward
    leader.setControl(velocityRequest.withVelocity(velocityRotations).withFeedForward(ffVolts));
  }

  /**
   * Set motor voltage directly.
   * 
   * @param voltage The voltage to apply
   */
  public void setVoltage(double voltage) {
    leader.setVoltage(voltage);
  }

  /**
   * Get the elevator simulation for testing.
   * 
   * @return The elevator simulation model
   */
  public ElevatorSim getSimulation() {
    return elevatorSim;
  }

  public double getMinHeightMeters() {
    return minHeight;
  }

  public double getMaxHeightMeters() {
    return maxHeight;
  }

  /**
   * Creates a command to set the elevator to a specific height.
   * 
   * @param heightMeters The target height in meters
   * @return A command that sets the elevator to the specified height
   */
  public Command setHeightCommand(double heightMeters) {
    return runOnce(() -> setPosition(heightMeters));
  }

  /**
   * Creates a command to move the elevator to a specific height with a profile.
   * 
   * @param heightMeters The target height in meters
   * @return A command that moves the elevator to the specified height
   */
  public Command moveToHeightCommand(double heightMeters) {
    return run(() -> {
      double currentHeight = getPositionMeters();
      double error = heightMeters - currentHeight;
      double velocity = Math.signum(error) * Math.min(Math.abs(error) * 2.0, maxVelocity);
      setVelocity(velocity);
    }).until(() -> {
      double currentHeight = getPositionMeters();
      return Math.abs(heightMeters - currentHeight) < 0.02; // 2cm tolerance
    });
  }

  public void moveWithJoysticks(double value) {
    double deadband = 0.05;
    if (Math.abs(value) < deadband) {
      setVelocity(0);
    } else {
      double velocity = value / maxVelocity;
      setVelocity(velocity);
    }
  }

  /**
   * Creates a command to stop the elevator.
   * 
   * @return A command that stops the elevator
   */
  public Command stopCommand() {
    return runOnce(() -> setVelocity(0));
  }

  /**
   * Creates a command to move the elevator at a specific velocity.
   * 
   * @param velocityMetersPerSecond The target velocity in meters per second
   * @return A command that moves the elevator at the specified velocity
   */
  public Command moveAtVelocityCommand(double velocityMetersPerSecond) {
    return run(() -> setVelocity(velocityMetersPerSecond));
  }
}