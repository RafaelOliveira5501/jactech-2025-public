package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionIOSim.TimestampedPose;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOPhotonVision extends PoseEstimator implements VisionIO {

  private final List<PhotonCamera> cameras = new ArrayList<>();
  private final List<Transform3d> robotToCameras = new ArrayList<>();
  private final AprilTagFieldLayout fieldLayout;
  private final Drive drive;

  public VisionIOPhotonVision(Drive drive) {
    this.drive = drive;
    this.fieldLayout = VisionConstants.APRILTAG_FIELD_LAYOUT;

    // Initialize cameras and their transforms
    for (String cameraName : VisionConstants.cameraNames) {
      PhotonCamera camera = new PhotonCamera(cameraName);
      cameras.add(camera);

      // Match each camera name to its transform
      switch (cameraName) {
        case VisionConstants.CAMERA_FRONT_LEFT:
          robotToCameras.add(VisionConstants.CAMERA_FRONT_LEFT_TRANSFORM);
          break;
        case VisionConstants.CAMERA_FRONT_RIGHT:
          robotToCameras.add(VisionConstants.CAMERA_FRONT_RIGHT_TRANSFORM);
          break;
        case VisionConstants.CAMERA_BACK_LEFT:
          robotToCameras.add(VisionConstants.CAMERA_BACK_LEFT_TRANSFORM);
          break;
        case VisionConstants.CAMERA_BACK_RIGHT:
          robotToCameras.add(VisionConstants.CAMERA_BACK_RIGHT_TRANSFORM);
          break;
        default:
          throw new IllegalArgumentException("Unknown camera: " + cameraName);
      }
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = true;
    inputs.poseObservations = new PoseObservation[0];
    List<EstimatedPose> estimatedPoses = new ArrayList<>();
    List<PoseObservation> observations = new ArrayList<>();

    for (int i = 0; i < cameras.size(); i++) {
      PhotonCamera camera = cameras.get(i);
      Transform3d robotToCamera = robotToCameras.get(i);

      // Check camera connection
      if (!camera.isConnected()) {
        inputs.connected = false;
        Logger.recordOutput("Vision/Camera" + i + "/Connected", false);
        continue;
      }
      Logger.recordOutput("Vision/Camera" + i + "/Connected", true);

      PhotonPipelineResult result = camera.getLatestResult();
      Logger.recordOutput("Vision/Camera" + camera.getName() + "/HasTargets", result.hasTargets());

      Logger.recordOutput("Vision/Arducam/alignmentTarget", inputs.alignmentTarget);
      Logger.recordOutput("Vision/Arducam/Confidence", inputs.ta);
      Logger.recordOutput("Vision/Arducam/HasTargets", result.hasTargets());

      if (camera.getName().equals("cam_FrontRight")) {
        PhotonTrackedTarget bestTarget = result.getBestTarget();

        if (bestTarget != null) { // Check for null before using it
          inputs.alignmentTarget = bestTarget.getFiducialId();
          inputs.ta = bestTarget.getArea(); // Confidence level
        } else {
          inputs.alignmentTarget = -1; // Reset if no tag is seen
          inputs.ta = 0.0; // Reset confidence level
        }
      }

      if (!result.hasTargets()) continue;

      double latencySecs = result.metadata.getLatencyMillis() / 1000.0;
      double timestamp = Timer.getFPGATimestamp() - latencySecs;
      for (PhotonTrackedTarget target : result.getTargets()) {
        int tagId = target.getFiducialId();
        Optional<Pose3d> tagPoseOpt = fieldLayout.getTagPose(tagId);
        if (tagPoseOpt.isEmpty()) {
          Logger.recordOutput("Vision/Camera" + i + "/TagNotFound", tagId);
          continue;
        }

        Pose3d tagPose = tagPoseOpt.get();
        Logger.recordOutput("Vision/Camera" + i + "/TagPose", tagPose);

        // Get historical pose at image capture time
        TimestampedPose historicalPose = drive.getHistoricalPose(timestamp);
        Pose3d historicalRobotPose = new Pose3d(historicalPose.pose);

        // Calculate camera pose using historical robot pose
        Pose3d historicalCameraPose = historicalRobotPose.transformBy(robotToCamera);

        // Calculate camera pose relative to tag
        Transform3d cameraToTarget = target.getBestCameraToTarget();
        Pose3d cameraPose = tagPose.transformBy(cameraToTarget.inverse());

        // Calculate robot pose from camera pose
        Transform3d cameraToRobot = robotToCamera.inverse();
        Pose3d robotPose = cameraPose.transformBy(cameraToRobot);

        // Check detection validity
        double distance =
            historicalCameraPose.getTranslation().getDistance(tagPose.getTranslation());
        if (distance > VisionConstants.MAX_DETECTION_DISTANCE) {
          Logger.recordOutput("Vision/Camera" + i + "/DistanceExceeded", distance);
          continue;
        }

        double ambiguity = target.getPoseAmbiguity();
        if (ambiguity >= VisionConstants.AMBIGUITY_THRESHOLD) {
          Logger.recordOutput("Vision/Camera" + i + "/AmbiguityExceeded", ambiguity);
          continue;
        }

        // Calculate weight and add to estimates
        double weight = calculateWeight(target, historicalCameraPose);
        estimatedPoses.add(new EstimatedPose(robotPose, weight, timestamp));

        // Create observation
        PoseObservation observation =
            new PoseObservation(
                timestamp,
                robotPose,
                latencySecs,
                ambiguity,
                target.getDetectedObjectConfidence(),
                1L << tagId,
                1,
                distance,
                PoseObservationType.SINGLE_TAG);

        Logger.recordOutput("Vision/Camera" + i + "/Observation", observation);
        observations.add(observation);
      }
    }

    inputs.poseObservations = observations.toArray(new PoseObservation[0]);

    // Run RANSAC on collected estimates
    RansacResult ransacResult =
        calculatePoseWithRANSAC(estimatedPoses, true, Timer.getFPGATimestamp(), drive.getPose());

    if (ransacResult != null && ransacResult.pose != null) {
      drive.addVisionMeasurement(
          ransacResult.pose.toPose2d(),
          ransacResult.timestamp,
          VisionConstants.VISION_MEASUREMENT_STD_DEVS);
    }

    // Estimate CPU usage (optional)
    estimateCpuUsage(inputs);
  }

  private void estimateCpuUsage(VisionIOInputs inputs) {
    // Placeholder for real CPU estimation logic
    inputs.cpuUsagePercent = 0.0;
    inputs.componentTimesMs = new double[] {0.0, 0.0, 0.0};
  }
}
