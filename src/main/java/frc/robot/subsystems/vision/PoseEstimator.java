package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.*;
import java.util.concurrent.*;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PoseEstimator {
  private static final int THREAD_POOL_SIZE = 4;

  public static class RansacResult {
    public final Pose3d pose;
    public final double timestamp;
    public final RansacIterationResult bestIteration;

    public RansacResult(Pose3d pose, double timestamp, RansacIterationResult bestIteration) {
      this.pose = pose;
      this.timestamp = timestamp;
      this.bestIteration = bestIteration;
    }
  }

  public RansacResult calculatePoseWithRANSAC(
      List<EstimatedPose> poses,
      boolean LM_perfectness,
      double timestamp,
      Pose2d currentDrivePose) {

    // Filter poses that deviate too far from odometry
    Pose3d odomPose = new Pose3d(currentDrivePose);
    List<EstimatedPose> filteredPoses =
        poses.stream().filter(ep -> ep.weight >= VisionConstants.MIN_WEIGHT_THRESHOLD).toList();

    // Modify return statements
    if (filteredPoses.isEmpty()) {
      Logger.recordOutput("Vision/RANSAC", "No poses to process.");
      return new RansacResult(null, timestamp, null);
    }

    final int maxIterations = VisionConstants.MAX_RANSAC_ITERATIONS;
    final double inlierThreshold = VisionConstants.RANSAC_INLIER_THRESHOLD;
    final int numPoses = filteredPoses.size();

    // Precompute pose data into arrays
    final double[] xCoords = new double[numPoses];
    final double[] yCoords = new double[numPoses];
    final double[] zCoords = new double[numPoses];
    final double[] quatW = new double[numPoses];
    final double[] quatX = new double[numPoses];
    final double[] quatY = new double[numPoses];
    final double[] quatZ = new double[numPoses];
    final double[] weights = new double[numPoses];

    for (int i = 0; i < numPoses; i++) {
      EstimatedPose ep = filteredPoses.get(i);
      Pose3d pose = ep.pose;
      xCoords[i] = pose.getX();
      yCoords[i] = pose.getY();
      zCoords[i] = pose.getZ();
      Quaternion q = pose.getRotation().getQuaternion();
      quatW[i] = q.getW();
      quatX[i] = q.getX();
      quatY[i] = q.getY();
      quatZ[i] = q.getZ();
      weights[i] = ep.weight;
    }

    ExecutorService executor = Executors.newFixedThreadPool(THREAD_POOL_SIZE);
    try {
      int iterationsPerTask = maxIterations / THREAD_POOL_SIZE;
      List<Future<RansacIterationResult>> futures = new ArrayList<>();

      for (int i = 0; i < THREAD_POOL_SIZE; i++) {
        final int start = i * iterationsPerTask;
        final int end = (i == THREAD_POOL_SIZE - 1) ? maxIterations : start + iterationsPerTask;
        futures.add(
            executor.submit(
                () -> {
                  RansacIterationResult bestResult = null;
                  for (int j = start; j < end; j++) {
                    RansacIterationResult result =
                        processRansacIteration(
                            numPoses,
                            inlierThreshold,
                            xCoords,
                            yCoords,
                            zCoords,
                            quatW,
                            quatX,
                            quatY,
                            quatZ,
                            weights);
                    if (bestResult == null || result.inlierCount > bestResult.inlierCount) {
                      bestResult = result;
                    }
                  }
                  if (bestResult != null) {
                    // Directly use the RANSAC result without refinement
                    bestResult =
                        new RansacIterationResult(
                            bestResult.inlierCount,
                            bestResult.pose, // Original pose
                            bestResult.inlierIndices,
                            bestResult.outlierIndices);
                  }
                  return bestResult;
                }));
      }

      executor.shutdown();
      executor.awaitTermination(2, TimeUnit.SECONDS);

      Pose3d bestPose = null;
      RansacIterationResult bestIterationResult = null;
      int bestInlierCount = 0;
      List<Integer> allOutlierIndices = new ArrayList<>();

      for (Future<RansacIterationResult> future : futures) {
        RansacIterationResult result = future.get();
        if (result != null && result.inlierCount > bestInlierCount) {
          bestInlierCount = result.inlierCount;
          bestPose = result.pose;
          bestIterationResult = result;
        }
        if (result != null) {
          allOutlierIndices.addAll(result.outlierIndices);
        }
      }

      logResults(bestPose, bestInlierCount, allOutlierIndices, filteredPoses, currentDrivePose);
      return new RansacResult(bestPose, timestamp, bestIterationResult); // Use median timestamp
    } catch (InterruptedException | ExecutionException e) {
      Thread.currentThread().interrupt();
      return null;
    }
  }

  private RansacIterationResult processRansacIteration(
      int numPoses,
      double inlierThreshold,
      double[] xCoords,
      double[] yCoords,
      double[] zCoords,
      double[] quatW,
      double[] quatX,
      double[] quatY,
      double[] quatZ,
      double[] weights) {
    int randomIndex = ThreadLocalRandom.current().nextInt(numPoses);
    double refX = xCoords[randomIndex];
    double refY = yCoords[randomIndex];
    double refZ = zCoords[randomIndex];
    double refQW = quatW[randomIndex];
    double refQX = quatX[randomIndex];
    double refQY = quatY[randomIndex];
    double refQZ = quatZ[randomIndex];

    List<Integer> inlierIndices = new ArrayList<>();
    List<Integer> outlierIndices = new ArrayList<>();

    for (int j = 0; j < numPoses; j++) {
      double dx = refX - xCoords[j];
      double dy = refY - yCoords[j];
      double dz = refZ - zCoords[j];
      double positionDistance =
          Math.sqrt(dx * dx + dy * dy + dz * dz) / VisionConstants.POSITION_NORM_SCALE;

      // Optional: Include rotation check
      double dot = refQW * quatW[j] + refQX * quatX[j] + refQY * quatY[j] + refQZ * quatZ[j];
      double angle = 2 * Math.acos(Math.abs(dot));
      double combinedDistance = positionDistance + angle * VisionConstants.ROTATION_WEIGHT;

      if (combinedDistance < inlierThreshold) {
        inlierIndices.add(j);
      } else {
        outlierIndices.add(j);
      }

      Logger.recordOutput("Vision/RANSAC/PositionDistance", positionDistance);
      Logger.recordOutput("Vision/RANSAC/RotationAngle", Math.toDegrees(angle));
      Logger.recordOutput("Vision/RANSAC/CombinedDistance", combinedDistance);
      Logger.recordOutput("Vision/RANSAC/Threshold", inlierThreshold);
    }

    return new RansacIterationResult(
        inlierIndices.size(),
        calculateWeightedAveragePose(
            inlierIndices,
            refQW,
            refQX,
            refQY,
            refQZ,
            xCoords,
            yCoords,
            zCoords,
            quatW,
            quatX,
            quatY,
            quatZ,
            weights),
        inlierIndices,
        outlierIndices);
  }

  private Pose3d calculateWeightedAveragePose(
      List<Integer> inlierIndices,
      double refQW,
      double refQX,
      double refQY,
      double refQZ,
      double[] xCoords,
      double[] yCoords,
      double[] zCoords,
      double[] quatW,
      double[] quatX,
      double[] quatY,
      double[] quatZ,
      double[] weights) {
    if (inlierIndices.isEmpty()) return null;

    double totalWeight = 0;
    double sumX = 0, sumY = 0, sumZ = 0;
    double sumQW = 0, sumQX = 0, sumQY = 0, sumQZ = 0;

    for (int index : inlierIndices) {
      double weight = weights[index];
      sumX += xCoords[index] * weight;
      sumY += yCoords[index] * weight;
      sumZ += zCoords[index] * weight;

      double qw = quatW[index];
      double qx = quatX[index];
      double qy = quatY[index];
      double qz = quatZ[index];

      // Flip quaternion if it's in the opposite hemisphere
      double dot = refQW * qw + refQX * qx + refQY * qy + refQZ * qz;
      if (dot < 0) {
        qw = -qw;
        qx = -qx;
        qy = -qy;
        qz = -qz;
      }

      sumQW += qw * weight;
      sumQX += qx * weight;
      sumQY += qy * weight;
      sumQZ += qz * weight;

      totalWeight += weight;
    }

    if (totalWeight == 0) return null;

    double avgX = sumX / totalWeight;
    double avgY = sumY / totalWeight;
    double avgZ = sumZ / totalWeight;

    double norm = Math.sqrt(sumQW * sumQW + sumQX * sumQX + sumQY * sumQY + sumQZ * sumQZ);
    if (norm == 0) return new Pose3d(avgX, avgY, avgZ, new Rotation3d());

    Quaternion avgQuat = new Quaternion(sumQW / norm, sumQX / norm, sumQY / norm, sumQZ / norm);
    return new Pose3d(avgX, avgY, avgZ, new Rotation3d(avgQuat));
  }

  public double calculateWeight(PhotonTrackedTarget target, Pose3d cameraPoseInField) {
    // Existing ambiguity weighting
    double ambiguityWeight = 1.0 / Math.max(target.getPoseAmbiguity(), 0.01);

    // Add distance-based weighting (example: inverse distance)
    double distance =
        cameraPoseInField
            .getTranslation()
            .getDistance(target.getBestCameraToTarget().getTranslation());
    double distanceWeight = 1.0 / (1.0 + distance);

    Translation3d camToTag = target.getBestCameraToTarget().getTranslation();
    double alignment = Math.abs(camToTag.getX() / camToTag.getNorm()); // [0-1]
    double alignmentWeight = 1.0 + alignment; // 1.0-2.0

    // Combine weights (could also add tag size weighting)
    return ambiguityWeight * distanceWeight * alignmentWeight;
  }

  private void logResults(
      Pose3d bestPose,
      int bestInlierCount,
      List<Integer> outlierIndices,
      List<EstimatedPose> originalPoses,
      Pose2d currentDrivePose) {
    Logger.recordOutput("Vision/RANSAC/BestInlierCount", bestInlierCount);
    if (bestPose != null) {
      Logger.recordOutput("Vision/RANSAC/BestPose", bestPose);
    }

    List<Pose3d> outliers = new ArrayList<>(outlierIndices.size());
    for (int index : outlierIndices) {
      outliers.add(originalPoses.get(index).pose);
    }
    Logger.recordOutput("Vision/RANSAC/Outliers", outliers.toArray(new Pose3d[0]));
    Logger.recordOutput(
        "Vision/RANSAC/InlierRatio", (double) bestInlierCount / originalPoses.size());

    // Log pose differences from odometry
    if (bestPose != null) {
      Pose3d odomPose = new Pose3d(currentDrivePose);
      Logger.recordOutput("Vision/OdometryDeltaX", bestPose.getX() - odomPose.getX());
      Logger.recordOutput("Vision/OdometryDeltaY", bestPose.getY() - odomPose.getY());
      Logger.recordOutput(
          "Vision/OdometryDeltaRot",
          bestPose.getRotation().minus(odomPose.getRotation()).getAngle());
    }
  }

  static class RansacIterationResult {
    final int inlierCount;
    final Pose3d pose;
    final List<Integer> inlierIndices;
    final List<Integer> outlierIndices;

    RansacIterationResult(
        int inlierCount, Pose3d pose, List<Integer> inlierIndices, List<Integer> outlierIndices) {
      this.inlierCount = inlierCount;
      this.pose = pose;
      this.inlierIndices = inlierIndices;
      this.outlierIndices = outlierIndices;
    }
  }

  static class EstimatedPose {
    public final Pose3d pose;
    public final double weight;
    public final double timestamp;

    public EstimatedPose(Pose3d pose, double weight, double timestamp) {
      this.pose = pose;
      this.weight = weight;
      this.timestamp = timestamp;
    }
  }
}
