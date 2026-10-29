package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;

public interface VisionIO {
  // Input structure to hold data for the vision system
  public static class VisionIOInputs {
    public boolean connected; // Is the camera connected
    public PoseObservation[] poseObservations = new PoseObservation[0]; // Array of detected poses
    public double tx;
    public double ty;
    public double ta;
    public double cpuUsagePercent;
    public double[] componentTimesMs = new double[3]; // [CameraProc, PoseEst, RANSAC]
    public int bestTarget;
    public int alignmentTarget;
  }

  // Represents a single vision pose observation
  public static record PoseObservation(
      double timestamp,
      Pose3d cameraPose,
      double latencySecs,
      double averageAmbiguity,
      double reprojectionError,
      long tagsSeenBitMap,
      int numTags,
      double averageTagDistance,
      PoseObservationType type) {}

  public enum PoseObservationType {
    SINGLE_TAG,
    MULTI_TAG
  }

  // Method to update the vision inputs
  public default void updateInputs(VisionIOInputs inputs) {}
}
