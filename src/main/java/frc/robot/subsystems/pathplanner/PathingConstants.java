package frc.robot.subsystems.pathplanner;

import com.pathplanner.lib.config.PIDConstants;

public final class PathingConstants {
  // Path constraints
  public static final double MAX_VELOCITY = 4.83; // m/s
  public static final double MAX_ACCEL = 4.0; // m/s²
  public static final double MAX_ANGULAR_VEL = Math.PI; // rad/s
  public static final double MAX_ANGULAR_ACCEL = 3 * Math.PI; // rad/s²
  public static final double PATHING_TIMEOUT = 10.0;

  // PID Constants
  public static final PIDConstants TRANSLATION_PID =
      new PIDConstants(
          2.0, // P (reduced from 5.0 - path following needs less than module control)
          0.0, // I
          0.05 // D
          );

  public static final PIDConstants ROTATION_PID =
      new PIDConstants(
          1.5, // P (reduced from 3.0 - matches your turnKp ratio)
          0.0, // I
          0.2 // D
          );

  // Path files (autocomplete-safe)
  public enum PathName {
    AMP_SCORE("AmpScore"),
    SPEAKER_SHOT("SpeakerShot"),
    STAGE_RETREAT("StageRetreat");

    public final String filename;

    PathName(String filename) {
      this.filename = filename;
    }
  }
}
