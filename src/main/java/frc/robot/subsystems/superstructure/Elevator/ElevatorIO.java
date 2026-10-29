package frc.robot.subsystems.superstructure.Elevator;

import org.littletonrobotics.junction.AutoLog;

/** Interface for hardware and simulation layers of the Elevator subsystem. */
public interface ElevatorIO {

  /** Container class for all sensor inputs of the elevator. */
  @AutoLog
  class ElevatorIOInputs {
    public boolean leaderMotorConnected = false; // State of the leader motor connection
    public boolean followerMotorConnected = false; // State of the follower motor connection
    public double positionMeters = 0.0; // Current position of the elevator (meters)
    public double velocityMetersPerSec = 0.0; // Current velocity of the elevator (meters/sec)
    public double appliedVolts; // Voltage applied to the elevator motor
    public double supplyCurrentAmps; // Current draw (amps)
    public double torqueCurrentAmps; // Torque-producing current (amps)
    public double tempCelcius; // Motor temperature (°C)
  }

  /**
   * Updates the given inputs object with the latest sensor readings. This is called periodically by
   * the subsystem.
   *
   * @param inputs The object to store sensor data.
   */
  default void updateInputs(ElevatorIOInputs inputs) {}

  /**
   * Run the elevator motors with a specific current (amps).
   *
   * @param amps The desired current to apply.
   */
  default void runCurrent(double amps) {}

  /** Stops all elevator motors immediately. */
  default void stop() {}

  /**
   * Enables or disables brake mode on the elevator motors.
   *
   * @param enable True to enable brake mode, false to disable.
   */
  default void setBrakeMode(boolean enable) {}

  /**
   * Run the elevator motors at a specific voltage.
   *
   * @param volts The voltage to apply to the motors.
   */
  default void runVolts(double volts) {}

  /**
   * Updates the PID gains for the elevator motors.
   *
   * @param p Proportional gain.
   * @param i Integral gain.
   * @param d Derivative gain.
   */
  default void setPID(double p, double i, double d) {}

  /**
   * Moves the elevator to a specific setpoint using position control.
   *
   * @param setpoint The target position (meters).
   * @param feedforward The feedforward value to assist with motion.
   */
  default void runSetpoint(double setpoint, double feedforward) {}
}
