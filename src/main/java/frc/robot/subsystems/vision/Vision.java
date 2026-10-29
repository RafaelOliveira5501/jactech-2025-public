package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIO visionIO;
  private final VisionIOInputs inputs = new VisionIOInputs();
  private final Field2d field = new Field2d(); // Field for visualization
  private VisionIOInputs arducamInputs;
  // private final VisionLimelight visionLimelight; // Store reference to Limelight system

  public Vision(VisionIO visionIO, Supplier<Pose3d> poseSupplier) {
    this.visionIO = visionIO;
    // this.visionLimelight = visionLimelight;
    SmartDashboard.putData("Field", field); // Add to the dashboard
  }

  @Override
  public void periodic() {
    visionIO.updateInputs(inputs);
    // visionLimelight.updateArducamInputs(inputs); // Send to Limelight subsystem

    // Log whether the camera is connected
    Logger.recordOutput("Vision/Connected", inputs.connected);
    Logger.recordOutput("Vision/Values/tX", inputs.tx);
    Logger.recordOutput("Vision/Values/ty", inputs.ty);

    if (inputs.connected) {
      // Create a list to store detected tag poses
      List<Pose3d> detectedTagPoseList = new ArrayList<>();
      List<Pose3d> tagsPoseList = new ArrayList<>();

      Logger.recordOutput("Vision/NumObservations", inputs.poseObservations.length);

      for (int i = 0; i < inputs.poseObservations.length; i++) {
        PoseObservation observation = inputs.poseObservations[i];
        Logger.recordOutput("Vision/Observation" + i, observation.cameraPose());

        tagsPoseList.add(observation.cameraPose());

        // Iterate over detected tags
        for (int tagID = 0; tagID < 30; tagID++) { // Adjust max tag count as needed
          if ((observation.tagsSeenBitMap() & (1L << tagID)) != 0) {
            // Add tag pose to the list
            detectedTagPoseList.add(observation.cameraPose());
          }
        }

        // Update Field2d visualization
        field.getObject("CameraPose" + i).setPose(observation.cameraPose().toPose2d());
      }

      // Convert the list of poses to an array
      Pose3d[] detectedTagPoses = detectedTagPoseList.toArray(new Pose3d[0]);
      Logger.recordOutput("Vision/DetectedTagPoses", detectedTagPoses);
    } else {
      Logger.recordOutput("Vision/Error", "Camera not connected.");
    }
  }

  public VisionIOInputs getInputs() {
    return this.inputs; // Return the VisionIOInputs instance
  }
}
