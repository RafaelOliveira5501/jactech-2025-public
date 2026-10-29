package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOLimelightSim implements VisionIO {
  private final VisionSystemSim simVisionSystem;
  private final PhotonCameraSim limelightSim;
  private final PhotonCamera limelight;
  private final Transform3d robotToLimelight;
  private final Drive drive;
  private final AprilTagFieldLayout fieldLayout;

  public VisionIOLimelightSim(
      Drive drive, AprilTagFieldLayout fieldLayout, Transform3d robotToLimelight) {
    this.drive = drive;
    this.fieldLayout = fieldLayout;
    this.robotToLimelight = robotToLimelight;

    // Create a simulated vision system (acts like PhotonVision)
    simVisionSystem = new VisionSystemSim("SimLimelight");
    simVisionSystem.addAprilTags(fieldLayout);

    // Create a simulated Limelight camera
    limelight = new PhotonCamera("SimLimelight");
    limelightSim = new PhotonCameraSim(limelight, createCameraProperties(), fieldLayout);

    // Attach Limelight to the simulated vision system
    simVisionSystem.addCamera(limelightSim, robotToLimelight);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Update the vision system using the current robot pose
    Pose2d robotPose = drive.getPose();
    simVisionSystem.update(robotPose);

    // Check if the Limelight "sees" any tags
    PhotonPipelineResult result = limelight.getLatestResult();
    inputs.connected = result.hasTargets();

    if (inputs.connected) {
      PhotonTrackedTarget bestTarget =
          result.getTargets().stream()
              .min(
                  (a, b) ->
                      Double.compare(
                          a.getBestCameraToTarget().getTranslation().getNorm(),
                          b.getBestCameraToTarget().getTranslation().getNorm()))
              .orElse(null);

      if (bestTarget == null) {
        inputs.connected = false;
        return;
      } else {
        inputs.bestTarget = bestTarget.fiducialId;
      }

      Optional<Pose3d> tagPoseOpt = fieldLayout.getTagPose(bestTarget.getFiducialId());

      if (tagPoseOpt.isPresent()) {
        Pose3d tagPose = tagPoseOpt.get();
        Transform3d cameraToTarget = bestTarget.getBestCameraToTarget();
        Pose3d limelightPose = tagPose.transformBy(cameraToTarget.inverse());

        // Convert Limelight Pose to Robot Pose
        Pose3d robotPoseFromVision = limelightPose.transformBy(robotToLimelight.inverse());

        // Simulate `tx`, `ty` like real Limelight
        inputs.tx = Math.toDegrees(cameraToTarget.getX());
        inputs.ty = Math.toDegrees(cameraToTarget.getY());

        // Log the simulated detection
        inputs.poseObservations =
            new PoseObservation[] {
              new PoseObservation(
                  Timer.getFPGATimestamp()
                      - Units.millisecondsToSeconds(result.metadata.getLatencyMillis()),
                  robotPoseFromVision,
                  Units.millisecondsToSeconds(result.metadata.getLatencyMillis()),
                  bestTarget.getPoseAmbiguity(),
                  0.0, // Placeholder for reprojection error
                  1L << bestTarget.getFiducialId(),
                  1,
                  tagPose.getTranslation().getNorm(),
                  PoseObservationType.SINGLE_TAG)
            };
      }
    }
  }

  private static SimCameraProperties createCameraProperties() {
    SimCameraProperties props = new SimCameraProperties();
    props.setCalibration(960, 720, Rotation2d.fromDegrees(70)); // Approximate Limelight FOV
    props.setFPS(90); // Limelight runs at 90Hz
    props.setAvgLatencyMs(30); // Approximate Limelight latency
    return props;
  }
}
