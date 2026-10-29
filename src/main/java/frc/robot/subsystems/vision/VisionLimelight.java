package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class VisionLimelight extends SubsystemBase {
  private final VisionIO visionIO;
  private final VisionIOInputs inputs = new VisionIOInputs();
  // private final Field2d field = new Field2d(); // Field for visualization
  private VisionIOInputs arducamInputs = new VisionIOInputs(); // Store Arducam vision data
  private boolean lastConnected = false;

  public VisionLimelight(VisionIO visionIO) {
    this.visionIO = visionIO;
  }

  @Override
  public void periodic() {
    visionIO.updateInputs(inputs);

    // Log whether the camera is connected
    if (inputs.connected != lastConnected) {
      Logger.recordOutput("Limelight/Connected", inputs.connected);
      lastConnected = inputs.connected;
    }

    if (!inputs.connected) {
      Logger.recordOutput("Limelight/Error", "Camera not connected.");
    }
  }

  public VisionIOInputs getInputs() {
    return this.inputs; // Return the VisionIOInputs instance
  }

  private Pose2d smoothedPose = new Pose2d(); // Start with a default pose

  public Pose2d getEstimatedPose() {
    if (inputs.poseObservations == null || inputs.poseObservations.length == 0) {
      System.out.println("[VisionLimelight] No pose estimate available.");
      return smoothedPose; // Return last known smoothed pose
    }

    Pose2d newPose = inputs.poseObservations[0].cameraPose().toPose2d();

    // Low-pass filter: Blend 90% old pose, 10% new pose
    double alpha = 0.1;
    smoothedPose =
        new Pose2d(
            smoothedPose.getX() * (1 - alpha) + newPose.getX() * alpha,
            smoothedPose.getY() * (1 - alpha) + newPose.getY() * alpha,
            new Rotation2d(
                smoothedPose.getRotation().getRadians() * (1 - alpha)
                    + newPose.getRotation().getRadians() * alpha));

    return smoothedPose;
  }

  public static Pose2d getLeftPose(Pose2d targetPose, double offset) {
    Rotation2d heading = targetPose.getRotation();
    return new Pose2d(
        targetPose.getX() - offset * Math.sin(heading.getRadians()), // Adjust X
        targetPose.getY() + offset * Math.cos(heading.getRadians()), // Adjust Y
        heading);
  }

  public static Pose2d getRightPose(Pose2d targetPose, double offset) {
    Rotation2d heading = targetPose.getRotation();
    return new Pose2d(
        targetPose.getX() + offset * Math.sin(heading.getRadians()), // Adjust X
        targetPose.getY() - offset * Math.cos(heading.getRadians()), // Adjust Y
        heading);
  }

  @SuppressWarnings("unchecked")
  public Optional<Supplier<Pose2d>[]> getTargetPoseForClosestTag() {

    boolean limelightSeesTarget = inputs.bestTarget != -1;
    boolean arducamSeesTarget = arducamInputs.connected && arducamInputs.bestTarget != -1;

    if (!limelightSeesTarget && !arducamSeesTarget) {
      // Logger.recordOutput("VisionLimelight/SelectedTargetSource", "None");
      return Optional.empty();
    }

    int tagID;
    if (limelightSeesTarget && arducamSeesTarget) {
      if (inputs.ta >= arducamInputs.ta) {
        Logger.recordOutput(
            "VisionLimelight/SelectedTargetSource", "Limelight (Higher Confidence)");
        tagID = inputs.bestTarget;
      } else {
        Logger.recordOutput("VisionLimelight/SelectedTargetSource", "Arducam (Higher Confidence)");
        tagID = arducamInputs.alignmentTarget;
      }
    } else if (arducamSeesTarget) {
      Logger.recordOutput("VisionLimelight/SelectedTargetSource", "Arducam (Only Arducam Sees)");
      tagID = arducamInputs.alignmentTarget;
    } else {
      Logger.recordOutput(
          "VisionLimelight/SelectedTargetSource", "Limelight (Only Limelight Sees)");
      tagID = inputs.bestTarget; // Default to Limelight if Arducam is missing
    }
    // Logger.recordOutput("VisionLimelight/FinalSelectedTag", tagID);

    Supplier<Pose2d> targetPose;
    if (tagID == -1) {
      System.out.println("[DEBUG] No valid tag detected. Returning empty.");
      return Optional.empty();
    }

    switch (tagID) {
        // Blue side
      case 18:
        targetPose = () -> AllianceFlipUtil.apply(FieldConstants.Reef.reefScoringPosition[0].get());
        break;
      case 19:
        targetPose = () -> AllianceFlipUtil.apply(FieldConstants.Reef.reefScoringPosition[1].get());
        break;
      case 20:
        targetPose = () -> AllianceFlipUtil.apply(FieldConstants.Reef.reefScoringPosition[2].get());
        break;
      case 21:
        targetPose = () -> AllianceFlipUtil.apply(FieldConstants.Reef.reefScoringPosition[3].get());
        break;
      case 22:
        targetPose = () -> AllianceFlipUtil.apply(FieldConstants.Reef.reefScoringPosition[4].get());
        break;
      case 17:
        targetPose = () -> AllianceFlipUtil.apply(FieldConstants.Reef.reefScoringPosition[5].get());
        break;

        // Red side
      case 7:
        targetPose = () -> AllianceFlipUtil.apply(FieldConstants.Reef.reefScoringPosition[0].get());
        break;
      case 6:
        targetPose = () -> AllianceFlipUtil.apply(FieldConstants.Reef.reefScoringPosition[1].get());
        break;
      case 11:
        targetPose = () -> AllianceFlipUtil.apply(FieldConstants.Reef.reefScoringPosition[2].get());
        break;
      case 10:
        targetPose = () -> AllianceFlipUtil.apply(FieldConstants.Reef.reefScoringPosition[3].get());
        break;
      case 9:
        targetPose = () -> AllianceFlipUtil.apply(FieldConstants.Reef.reefScoringPosition[4].get());
        break;
      case 8:
        targetPose = () -> AllianceFlipUtil.apply(FieldConstants.Reef.reefScoringPosition[5].get());
        break;
      default:
        return Optional.empty(); // No valid tag detected
    }

    Supplier<Pose2d> leftPose = () -> getLeftPose(targetPose.get(), VisionConstants.LATERAL_OFFSET);
    Supplier<Pose2d> rightPose =
        () -> getRightPose(targetPose.get(), VisionConstants.LATERAL_OFFSET);

    // Logger.recordOutput("VisionLimelight/Switch/TargetPose", targetPose.get());
    Logger.recordOutput("VisionLimelight/Switch/LeftPose", leftPose.get());
    Logger.recordOutput("VisionLimelight/Switch/RightPose", rightPose.get());
    return Optional.of(new Supplier[] {targetPose, leftPose, rightPose});
  }

  public void updateArducamInputs(VisionIOInputs arducamInputs) {
    this.arducamInputs = arducamInputs; // Copy Arducam inputs
  }
}
