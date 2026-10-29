package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;

/** IO implementation for real Limelight hardware. */
public class VisionIOLimelight implements VisionIO {

  private final String limelightName;

  /**
   * Creates a new VisionIOLimelight.
   *
   * @param name The configured name of the Limelight.
   */
  public VisionIOLimelight(String name) {
    this.limelightName = name;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Check if the Limelight has a valid target
    inputs.connected = LimelightHelpers.getTV(limelightName);

    if (!inputs.connected) {
      inputs.bestTarget = -1; // Reset best target if no tag is seen
      inputs.ta = 0.0; // Reset confidence level
      return; // Skip the rest
    }

    if (!inputs.connected) {
      System.out.println("[VisionIOLimelight] No valid connection to Limelight.");
      return;
    }

    // Read Limelight data using LimelightHelpers
    inputs.tx = LimelightHelpers.getTX(limelightName);
    inputs.ty = LimelightHelpers.getTY(limelightName);
    inputs.ta = LimelightHelpers.getTA(limelightName);
    double latencyMs =
        LimelightHelpers.getLatency_Pipeline(limelightName)
            + LimelightHelpers.getLatency_Capture(limelightName);

    // Get the detected AprilTag ID using LimelightHelpers
    int tagID = (int) LimelightHelpers.getFiducialID(limelightName);
    if (tagID == -1) {
      System.out.println("[VisionIOLimelight] No valid tag detected.");
      return;
    }
    inputs.bestTarget = tagID;

    // Get the robot's estimated pose from Limelight
    Pose3d estimatedPose =
        LimelightHelpers.getBotPose3d_wpiBlue(limelightName); // Use Red for red alliance

    // Store pose observation
    inputs.poseObservations =
        new PoseObservation[] {
          new PoseObservation(
              RobotController.getFPGATime() / 1.0e6 - (latencyMs / 1000.0),
              estimatedPose,
              latencyMs / 1000.0,
              0.0, // Pose ambiguity (not available from Limelight)
              0.0, // Placeholder reprojection error
              1L, // Assume one tag detected
              1,
              estimatedPose.getTranslation().getZ(), // Distance from Limelight
              PoseObservationType.SINGLE_TAG)
        };
  }
}
