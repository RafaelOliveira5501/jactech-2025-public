package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.Drive;
import java.util.*;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.*;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOSim extends PoseEstimator implements VisionIO {
  // Reusable objects to minimize allocations
  private Pose3d tempPose = new Pose3d();

  // Pre-allocated collections with typical capacities
  private final List<Pose3d> cameraPosesInField = new ArrayList<>(4);
  private final List<PoseObservation> observations = new ArrayList<>(20);
  private final List<EstimatedPose> estimatedPoses = new ArrayList<>(20);
  private final Map<Integer, Pose3d> estimatedPosesByCamera = new HashMap<>(4);

  // Simulation components
  private final VisionSystemSim simVisionSystem;
  private final AprilTagFieldLayout fieldLayout;
  private final List<PhotonCameraSim> cameraSims = new ArrayList<>();
  private final List<PhotonCamera> photonCameras = new ArrayList<>();
  private final List<Transform3d> robotToCameras = new ArrayList<>();
  private final Supplier<Pose2d> poseSupplier;
  private final Drive drive;

  public VisionIOSim(
      AprilTagFieldLayout fieldLayout,
      Supplier<Pose2d> poseSupplier,
      List<Transform3d> robotToCameras,
      Drive drive) {
    this.fieldLayout = fieldLayout;
    this.poseSupplier = poseSupplier;
    this.drive = drive;
    this.robotToCameras.addAll(robotToCameras);

    // Initialize vision simulation
    simVisionSystem = new VisionSystemSim("SimVisionSystem");
    simVisionSystem.addAprilTags(fieldLayout);

    // Configure simulated cameras
    for (int i = 0; i < robotToCameras.size(); i++) {
      PhotonCamera photonCamera = new PhotonCamera("SimPhotonCamera" + i);
      photonCameras.add(photonCamera);

      SimCameraProperties cameraProps = createCameraProperties();
      PhotonCameraSim cameraSim = new PhotonCameraSim(photonCamera, cameraProps, fieldLayout);
      cameraSims.add(cameraSim);

      simVisionSystem.addCamera(cameraSim, robotToCameras.get(i));
    }
  }

  private SimCameraProperties createCameraProperties() {
    SimCameraProperties props = new SimCameraProperties();
    props.setCalibration(1280, 720, Rotation2d.fromDegrees(60)); // Reduced resolution
    props.setFPS(60); // Match real hardware
    props.setAvgLatencyMs(50); // Typical latency
    props.setLatencyStdDevMs(0); // Reduced jitter
    return props;
  }

  private void estimateCpuUsage(VisionIOInputs inputs) {
    // 1. Camera Processing Time
    int totalTargets = observations.size();
    double cameraTimeMs = totalTargets * 0.05; // 50μs per target

    // 2. Pose Estimation Time
    double poseEstTimeMs = estimatedPoses.size() * 0.02; // 20μs per pose

    // 3. RANSAC Time
    int iterations = VisionConstants.MAX_RANSAC_ITERATIONS;
    double ransacTimeMs = iterations * VisionConstants.RANSAC_ITER_TIME_NS / 1e6;
    ransacTimeMs /= VisionConstants.CPU_CORES; // Parallelized across cores

    // Total cycle time (assumes 50Hz update rate)
    double totalTimeMs = cameraTimeMs + poseEstTimeMs + ransacTimeMs;
    double cycleTimeMs = 1000.0 / 50.0; // 20ms for 50Hz loop

    inputs.cpuUsagePercent = (totalTimeMs / cycleTimeMs) * 100;
    inputs.componentTimesMs = new double[] {cameraTimeMs, poseEstTimeMs, ransacTimeMs};

    Logger.recordOutput("Vision/CPUUsage", inputs.cpuUsagePercent);
    Logger.recordOutput("Vision/ComponentTimes", inputs.componentTimesMs);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    final Pose2d robotPose = poseSupplier.get();
    simVisionSystem.update(robotPose);
    inputs.connected = true;

    // Reset reusable containers
    cameraPosesInField.clear();
    observations.clear();
    estimatedPoses.clear();
    estimatedPosesByCamera.clear();

    processAllCameras(robotPose);
    processResults();

    inputs.poseObservations = observations.toArray(new PoseObservation[0]);

    estimateCpuUsage(inputs);
  }

  private void processAllCameras(Pose2d robotPose) {
    final Pose3d robotPose3d = new Pose3d(robotPose);

    for (int camIdx = 0; camIdx < photonCameras.size(); camIdx++) {
      final PhotonCamera camera = photonCameras.get(camIdx);
      final PhotonPipelineResult result = camera.getLatestResult();

      // Calculate camera pose once per camera
      tempPose = robotPose3d.transformBy(robotToCameras.get(camIdx));
      cameraPosesInField.add(
          new Pose3d(tempPose.getTranslation(), tempPose.getRotation())); // Safe copy

      if (result.hasTargets()) {
        processTargets(
            result.getTargets(),
            camIdx,
            tempPose,
            result.metadata.getLatencyMillis()); // Pass latency
      }
    }
  }

  private void processTargets(
      List<PhotonTrackedTarget> targets,
      int cameraIndex,
      Pose3d cameraPoseInField,
      double latencyMs) {
    for (PhotonTrackedTarget target : targets) {
      final Optional<Pose3d> tagPoseOpt = fieldLayout.getTagPose(target.getFiducialId());
      if (tagPoseOpt.isEmpty()) continue;

      final Pose3d tagPose = tagPoseOpt.get();
      final double distance = calculateDistance(cameraPoseInField, tagPose);

      if (isValidTarget(target, distance)) {
        processValidTarget(target, cameraIndex, tagPose, latencyMs);
      }
    }
  }

  private double calculateDistance(Pose3d cameraPose, Pose3d tagPose) {
    final Translation3d cameraTrans = cameraPose.getTranslation();
    final Translation3d tagTrans = tagPose.getTranslation();

    final double dx = cameraTrans.getX() - tagTrans.getX();
    final double dy = cameraTrans.getY() - tagTrans.getY();
    final double dz = cameraTrans.getZ() - tagTrans.getZ();

    return Math.sqrt(dx * dx + dy * dy + dz * dz);
  }

  private boolean isValidTarget(PhotonTrackedTarget target, double distance) {
    return distance <= VisionConstants.MAX_DETECTION_DISTANCE
        && target.getPoseAmbiguity() < VisionConstants.AMBIGUITY_THRESHOLD;
  }

  public static class TimestampedPose {
    public final double timestamp;
    public final Pose2d pose;

    public TimestampedPose(double timestamp, Pose2d pose) {
      this.timestamp = timestamp;
      this.pose = pose;
    }
  }

  private void processValidTarget(
      PhotonTrackedTarget target, int cameraIndex, Pose3d tagPose, double latencyMs) {
    // Calculate timestamp when image was captured
    double imageCaptureTimestamp =
        Timer.getFPGATimestamp() - Units.millisecondsToSeconds(latencyMs);

    // Get historical pose with timestamp
    TimestampedPose historicalPose = drive.getHistoricalPose(imageCaptureTimestamp);
    Pose3d historicalRobotPose3d = new Pose3d(historicalPose.pose);

    // Calculate camera pose relative to historical robot pose
    Pose3d historicalCameraPose =
        historicalRobotPose3d.transformBy(robotToCameras.get(cameraIndex));

    // Calculate vision-based robot pose using historical data
    Transform3d cameraToRobot = robotToCameras.get(cameraIndex).inverse();
    tempPose =
        tagPose
            .transformBy(target.getBestCameraToTarget().inverse()) // Camera pose in field
            .transformBy(cameraToRobot); // Apply camera-to-robot transform

    // Calculate weight using historical camera position
    final double weight = calculateWeight(target, historicalCameraPose);

    // Store with precise timestamp
    estimatedPoses.add(new EstimatedPose(tempPose, weight, imageCaptureTimestamp));
    estimatedPosesByCamera.put(cameraIndex, tempPose);

    // Log latency compensation details
    Logger.recordOutput(
        "Vision/LatencyCompensation/HistoricalPoseAge",
        imageCaptureTimestamp - historicalPose.timestamp);

    observations.add(createPoseObservation(target, tagPose, latencyMs));
  }

  private PoseObservation createPoseObservation(
      PhotonTrackedTarget target, Pose3d tagPose, double latencyMs) {
    return new PoseObservation(
        Timer.getFPGATimestamp() - Units.millisecondsToSeconds(latencyMs),
        tagPose,
        Units.millisecondsToSeconds(latencyMs),
        target.getPoseAmbiguity(),
        0.0, // Placeholder for reprojection error
        1L << target.getFiducialId(),
        1, // Single tag
        tagPose.getTranslation().getNorm(),
        PoseObservationType.SINGLE_TAG);
  }

  private void processResults() {
    // Batch logging
    estimatedPosesByCamera.forEach(
        (idx, pose) -> Logger.recordOutput("Vision/EstimatedRobotPoseByCamera" + idx, pose));

    // Calculate median timestamp from valid poses
    double[] timestamps =
        estimatedPoses.stream().mapToDouble(ep -> ep.timestamp).sorted().toArray();
    double medianTimestamp =
        timestamps.length > 0 ? timestamps[timestamps.length / 2] : Timer.getFPGATimestamp();

    // RANSAC processing
    final RansacResult result =
        calculatePoseWithRANSAC(estimatedPoses, true, medianTimestamp, drive.getPose());
    if (result.pose != null) {
      // Use median timestamp for odometry
      drive.addVisionMeasurement(
          result.pose.toPose2d(), medianTimestamp, VisionConstants.VISION_MEASUREMENT_STD_DEVS);

      // Logging
      Logger.recordOutput("Vision/PoseBeforeAdding", result.pose);
      Logger.recordOutput("Vision/BestInliers", result.bestIteration.inlierCount);
      Logger.recordOutput("Vision/BestRotation", result.bestIteration.pose.getRotation());
      Logger.recordOutput(
          "Odometry/TranslationError",
          drive.getPose().getTranslation().getDistance(result.pose.toPose2d().getTranslation()));
      Logger.recordOutput(
          "Odometry/RotationError",
          Math.abs(
              drive.getPose().getRotation().getDegrees()
                  - result.pose.toPose2d().getRotation().getDegrees()));
    }
  }

  @Override
  public double calculateWeight(PhotonTrackedTarget target, Pose3d cameraPoseInField) {
    final double ambiguityWeight = 1.0 / Math.max(target.getPoseAmbiguity(), 0.01);
    final double distance =
        cameraPoseInField
            .getTranslation()
            .getDistance(fieldLayout.getTagPose(target.getFiducialId()).get().getTranslation());
    final double distanceWeight = 1.0 / (1.0 + distance);

    return ambiguityWeight * distanceWeight;
  }
}
