package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import java.util.List;
import org.apache.commons.math3.linear.DecompositionSolver;
import org.apache.commons.math3.linear.LUDecomposition;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;

public class PoseRefiner {

  public Pose3d refinePose(
      Pose3d initialPose,
      List<Integer> inlierIndices,
      double[] xCoords,
      double[] yCoords,
      double[] zCoords,
      double[] quatW,
      double[] quatX,
      double[] quatY,
      double[] quatZ,
      double[] weights) {
    return refineWithLevenbergMarquardt(
        initialPose, inlierIndices, xCoords, yCoords, zCoords, quatW, quatX, quatY, quatZ, weights);
  }

  private Pose3d refineWithLevenbergMarquardt(
      Pose3d initialPose,
      List<Integer> inlierIndices,
      double[] xCoords,
      double[] yCoords,
      double[] zCoords,
      double[] quatW,
      double[] quatX,
      double[] quatY,
      double[] quatZ,
      double[] weights) {
    final double lambdaInit = 0.001;
    final int maxIterations = 10;
    final double epsilon = 1e-6;

    double[] params = poseToParameterArray(initialPose);
    double lambda = lambdaInit;
    double prevCost =
        computeCost(
            params, inlierIndices, xCoords, yCoords, zCoords, quatW, quatX, quatY, quatZ, weights);
    for (int iter = 0; iter < maxIterations; iter++) {
      // Compute Jacobian numerically
      double[][] J = new double[6][inlierIndices.size() * 6];
      double[] residuals =
          computeResiduals(
              params,
              inlierIndices,
              xCoords,
              yCoords,
              zCoords,
              quatW,
              quatX,
              quatY,
              quatZ,
              weights);

      for (int i = 0; i < 6; i++) { // For each parameter
        double[] perturbedParams = params.clone();
        perturbedParams[i] += epsilon;
        double[] perturbedResiduals =
            computeResiduals(
                perturbedParams,
                inlierIndices,
                xCoords,
                yCoords,
                zCoords,
                quatW,
                quatX,
                quatY,
                quatZ,
                weights);

        for (int j = 0; j < perturbedResiduals.length; j++) {
          J[i][j] = (perturbedResiduals[j] - residuals[j]) / epsilon;
        }
      }

      // Build weighted system: J^T W J + λ diag(J^T W J)
      RealMatrix JMat = MatrixUtils.createRealMatrix(J);
      RealMatrix W = computeWeightMatrix(inlierIndices, weights);
      RealMatrix JtW = JMat.transpose().multiply(W);
      RealMatrix JtWJ = JtW.multiply(JMat);
      double[] diagEntries = new double[JtWJ.getRowDimension()];
      for (int i = 0; i < JtWJ.getRowDimension(); i++) {
        diagEntries[i] = JtWJ.getEntry(i, i);
      }
      RealMatrix diag = MatrixUtils.createRealDiagonalMatrix(diagEntries);
      RealMatrix A = JtWJ.add(diag.scalarMultiply(lambda));

      // Compute -J^T W r
      RealVector rVec = MatrixUtils.createRealVector(residuals);
      RealVector B = JtW.operate(rVec).mapMultiply(-1);

      // Solve for Δ
      DecompositionSolver solver = new LUDecomposition(A).getSolver();
      RealVector delta = solver.solve(B);

      // Update parameters
      double[] newParams = params.clone();
      for (int i = 0; i < params.length; i++) {
        newParams[i] += delta.getEntry(i);
      }

      // Check cost improvement
      double newCost =
          computeCost(
              newParams,
              inlierIndices,
              xCoords,
              yCoords,
              zCoords,
              quatW,
              quatX,
              quatY,
              quatZ,
              weights);
      if (newCost < prevCost) {
        params = newParams;
        prevCost = newCost;
        lambda *= 0.1;
      } else {
        lambda *= 10.0;
      }
    }

    return parameterArrayToPose(params);
  }

  private RealMatrix computeWeightMatrix(List<Integer> inlierIndices, double[] weights) {
    int size = inlierIndices.size() * 6; // 3 translation + 3 rotation residuals per inlier
    double[] diagonal = new double[size];
    int idx = 0;
    for (int i : inlierIndices) {
      double weight = weights[i];
      // Apply weight to translation and rotation residuals
      for (int j = 0; j < 6; j++) {
        diagonal[idx++] = weight;
      }
    }
    return MatrixUtils.createRealDiagonalMatrix(diagonal);
  }

  private double[] poseToParameterArray(Pose3d pose) {
    // Get rotation vector as Vector<N3>
    var rotVec = pose.getRotation().getQuaternion().toRotationVector();
    return new double[] {
      pose.getX(),
      pose.getY(),
      pose.getZ(),
      rotVec.get(0), // Use index-based access
      rotVec.get(1),
      rotVec.get(2)
    };
  }

  private Pose3d parameterArrayToPose(double[] params) {
    // Convert back to axis-angle (rotation vector)
    Rotation3d rotation =
        new Rotation3d(
            params[3], // X component of rotation vector
            params[4], // Y component
            params[5] // Z component
            );
    return new Pose3d(params[0], params[1], params[2], rotation);
  }

  private double[] computeResiduals(
      double[] params,
      List<Integer> inlierIndices,
      double[] xCoords,
      double[] yCoords,
      double[] zCoords,
      double[] quatW,
      double[] quatX,
      double[] quatY,
      double[] quatZ,
      double[] weights) {
    Pose3d currentPose = parameterArrayToPose(params);
    double[] residuals = new double[inlierIndices.size() * 6];
    int idx = 0;

    for (int i : inlierIndices) {
      // Translation residual
      residuals[idx++] = (currentPose.getX() - xCoords[i]) * weights[i];
      residuals[idx++] = (currentPose.getY() - yCoords[i]) * weights[i];
      residuals[idx++] = (currentPose.getZ() - zCoords[i]) * weights[i];

      // Rotation residual (axis-angle difference)
      Quaternion qCurrent = currentPose.getRotation().getQuaternion();
      Quaternion qInlier = new Quaternion(quatW[i], quatX[i], quatY[i], quatZ[i]);
      Quaternion qDiff = qCurrent.inverse().times(qInlier);
      var rotVec = qDiff.toRotationVector();
      residuals[idx++] = rotVec.get(0) * weights[i]; // X
      residuals[idx++] = rotVec.get(1) * weights[i]; // Y
      residuals[idx++] = rotVec.get(2) * weights[i]; // Z
    }

    return residuals;
  }

  private double computeCost(
      double[] params,
      List<Integer> inlierIndices,
      double[] xCoords,
      double[] yCoords,
      double[] zCoords,
      double[] quatW,
      double[] quatX,
      double[] quatY,
      double[] quatZ,
      double[] weights) {
    double[] residuals =
        computeResiduals(
            params, inlierIndices, xCoords, yCoords, zCoords, quatW, quatX, quatY, quatZ, weights);
    double cost = 0.0;
    for (double r : residuals) {
      cost += r * r;
    }
    return cost;
  }
}
