package frc.robot.subsystems.superstructure.Elevator;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Constants for the Elevator subsystem. */
public class ElevatorConstants {
  // Motor IDs
  public static final int leaderElevatorCanId = 23;
  public static final int followerElevatorCanId = 24;

  // Motion magic and motor limits
  public static final int motionMagicSlot = 1;
  public static final int currentLimit = 40;

  // Gear reduction: output rotations per motor rotation
  public static final double reduction = 1 / 125.0;

  // Physical constraints
  public static final double maxLengthMeters = 0.15; // Maximum height of the elevator
  public static final double minLengthMeters = 0.0; // Maximum height of the elevator
  public static final double positionToleranceMeters = 0.0025; // Tolerance in meters (2.5 cm)

  // Feedforward gains (unused for now)
  public static final Gains gains = new Gains(4.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.17);

  // Elevator geometry
  public static final Translation2d elevatorOrigin = new Translation2d(-0.238, 0.298);

  // Motion profile constraints
  public static TrapezoidProfile.Constraints profileConstraints =
      new TrapezoidProfile.Constraints(6.0 * Math.PI, 20.0); // Max velocity and acceleration

  // Pulley diameter (meters)
  private static final double pulleyDiameterMeters = 0.05; // 5 cm

  /**
   * Converts motor rotations to linear distance in meters.
   *
   * @param rotations Motor rotations
   * @return Linear distance in meters
   */
  public static double rotationsToMeters(double rotations) {
    return rotations * Math.PI * pulleyDiameterMeters * reduction;
  }

  /**
   * Converts linear distance in meters to motor rotations.
   *
   * @param meters Linear distance in meters
   * @return Motor rotations
   */
  public static double metersToRotations(double meters) {
    return meters / (Math.PI * pulleyDiameterMeters * reduction);
  }

  // Record for PID and feedforward gains
  public record Gains(
      double kP, double kI, double kD, double ffkS, double ffkV, double ffkA, double ffkG) {}
}
