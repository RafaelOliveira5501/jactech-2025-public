package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.nio.file.Path;
import java.util.List;

public final class VisionConstants {

  // Camera Names (set these to match the names in PhotonVision UI)
  public static final String CAMERA_FRONT_LEFT = "cam_FrontLeft";
  public static final String CAMERA_FRONT_RIGHT = "cam_FrontRight";
  public static final String CAMERA_BACK_LEFT = "cam_BackLeft";
  public static final String CAMERA_BACK_RIGHT = "cam_BackRight";

  public static final List<String> cameraNames =
      List.of(CAMERA_FRONT_LEFT, CAMERA_FRONT_RIGHT, CAMERA_BACK_LEFT, CAMERA_BACK_RIGHT);

  // Camera-to-robot transforms (adjust these based on the camera positions)
  public static final Transform3d CAMERA_FRONT_LEFT_TRANSFORM =
      new Transform3d(
          new Translation3d(0.3, 0.3, 0.28),
          new Rotation3d(0.0, Math.toRadians(-20), Math.toRadians(90))); // Front left

  public static final Transform3d CAMERA_FRONT_RIGHT_TRANSFORM =
      new Transform3d(
          new Translation3d(0.3, -0.3, 0.28),
          new Rotation3d(0.0, Math.toRadians(-20), Math.toRadians(0))); // Front right

  public static final Transform3d CAMERA_BACK_LEFT_TRANSFORM =
      new Transform3d(
          new Translation3d(-0.3, 0.3, 0.28),
          new Rotation3d(0.0, Math.toRadians(-20), Math.toRadians(180))); // Back left

  public static final Transform3d CAMERA_BACK_RIGHT_TRANSFORM =
      new Transform3d(
          new Translation3d(-0.3, -0.3, 0.28),
          new Rotation3d(0.0, Math.toRadians(-20), Math.toRadians(270))); // Back right

  // Path to the field layout file (deploy/game.json)
  public static final Path APRILTAG_FIELD_LAYOUT_PATH =
      new File(Filesystem.getDeployDirectory(), "2025-reefscape.json").toPath();

  // Field layout for AprilTags
  public static final AprilTagFieldLayout APRILTAG_FIELD_LAYOUT = loadAprilTagFieldLayout();

  // Detection thresholds
  public static final double MAX_DETECTION_DISTANCE = 15.0; // Max distance in meters
  public static final double AMBIGUITY_THRESHOLD = 0.15; // Ambiguity threshold

  // RANSAC params
  public static final int MAX_RANSAC_ITERATIONS =
      400; // Iterations for RANSAC filter (higher = precision, lower = less cpu usage) default = 50
  public static final double POSITION_NORM_SCALE = 1.0; // Meters, typical position range
  public static final double RANSAC_INLIER_THRESHOLD =
      0.05; // Max distance (meters) to consider an observation as an inlier
  public static final double MIN_WEIGHT_THRESHOLD = 0.7;
  public static final double ROTATION_WEIGHT = 2.0;

  // Simulation Constants below here

  // Hardware specs (Orange Pi 5 Pro) - changeg to appropriate hardware
  public static final int CPU_CORES = 8;
  public static final double CLOCK_GHZ = 2.4;
  public static final double IPS_PER_CORE = 35000; // ARM Cortex-A76 estimate
  public static final double RANSAC_ITER_TIME_NS = 1500;
  public static final double THREAD_OVERHEAD_FACTOR = 0.15;

  public static final Matrix<N3, N1> VISION_MEASUREMENT_STD_DEVS = VecBuilder.fill(0.1, 0.1, 0.1);
  public static final Transform3d LIMELIGHT_TRANSFORM =
      new Transform3d(
          new Translation3d(-0.4, 0.35, 0.5), // X, Y, Z (meters)
          new Rotation3d(0.0, Units.degreesToRadians(0.0), 0.0) // Pitch downwards
          );

  public static final double LATERAL_OFFSET = 0.18;

  private VisionConstants() {
    // Private constructor to prevent instantiation
  }

  // Load the AprilTag field layout from the JSON file
  private static AprilTagFieldLayout loadAprilTagFieldLayout() {
    try {
      return new AprilTagFieldLayout(APRILTAG_FIELD_LAYOUT_PATH);
    } catch (Exception e) {
      e.printStackTrace();
      throw new RuntimeException(
          "Failed to load AprilTagFieldLayout: " + APRILTAG_FIELD_LAYOUT_PATH);
    }
  }
}
