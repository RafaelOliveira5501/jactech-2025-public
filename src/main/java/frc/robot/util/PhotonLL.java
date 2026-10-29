package frc.robot.util;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonLL extends SubsystemBase {
  public PhotonCamera FrontCamera = new PhotonCamera("Camera_Module_v1");
  public PhotonCamera SideLeftCamera = new PhotonCamera("Camera_Left");
  public PhotonCamera cam = FrontCamera;

  private double yaw;

  private double pitch;

  private double area;

  private double Id;

  private double xMeters;

  private double yMeters;

  private double zRotation;

  private boolean hasTargets;

  public boolean rangeControl;

  public boolean isAligned;

  public double visionControl;

  public PhotonPipelineResult getLatestResult() {
    return FrontCamera.getLatestResult();
  }

  public PhotonLL limelight;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var result = FrontCamera.getLatestResult();
    hasTargets = result.hasTargets();
    if (hasTargets) {
      var target = result.getBestTarget();

      // GET DATA:
      yaw = target.getYaw();
      pitch = target.getPitch();
      area = target.getArea();
      Id = target.getFiducialId();
      Transform3d camToTarget = target.getBestCameraToTarget();

      xMeters = camToTarget.getX();
      yMeters = camToTarget.getY();
      zRotation = camToTarget.getRotation().getZ();
    }
  }

  /**
   * Is the camera connected?
   *
   * @return Is the camera connected?
   */
  final boolean isCameraConnected() {
    return cam.isConnected();
  }

  private static PhotonLL instance;

  public static PhotonLL getInstance() {
    if (instance == null) {
      instance = new PhotonLL();
    }

    return instance;
  }

  public boolean hasValueTargets() {
    return hasTargets;
  }

  public double getYaw() {
    return yaw;
  }

  public double getYDistance() {
    return yMeters;
  }

  public Boolean correctID(int at) {
    if (Id == at) {
      return true;
    } else {
      return false;
    }
  }

  public double getXDistance() {
    return xMeters;
  }

  public double getZRotation() {
    return zRotation;
  }

  public double getArea() {
    return area;
  }

  public void filterAprilTags() {
    var result = FrontCamera.getLatestResult();

    if (result.hasTargets()) {
      // LIST of targets photon vision has
      var targets = result.getTargets();

      // checks to see if there is a list of apriltags to check. if no targets are visable, end
      // command
      if (targets.isEmpty()) {
        SmartDashboard.putBoolean("done", true);
      }

      var foundTargets =
          targets.stream()
              .filter(t -> t.getFiducialId() == 4 || t.getFiducialId() == 8)
              .filter(t -> !t.equals(4) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
              .findFirst();

      if (foundTargets.isPresent()) {
        var cameraToTarget = foundTargets.get().getBestCameraToTarget();
      }
    }
  }
}
