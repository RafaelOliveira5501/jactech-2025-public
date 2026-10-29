package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionLimelight;

public class StrafeToTarget extends Command {
  private final Drive drive;
  private final VisionLimelight vision;

  private static final double kP = 0.01; // Proportional control for strafing
  private static final double tolerance = 1; // Stop when |tx| < 1 degree
  private static final double maxSpeed = 0.4; // Limit strafe speed
  private final double desiredOffset;

  public StrafeToTarget(Drive drive, VisionLimelight vision, double offset) {
    this.drive = drive;
    this.vision = vision;
    this.desiredOffset = offset;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    double tx = vision.getInputs().tx; // Get horizontal offset from Limelight

    if (!vision.getInputs().connected) {
      drive.stop(); // Stop if no vision data
      return;
    }

    // P-Control: Calculate strafe speed (left/right movement)
    double error = desiredOffset - tx;
    double strafeSpeed = kP * error;

    // Clamp speed within limits
    strafeSpeed = Math.max(-maxSpeed, Math.min(maxSpeed, strafeSpeed));

    // Debug print
    System.out.println(
        "[StrafeToTarget] tx: "
            + tx
            + " | Desired: "
            + desiredOffset
            + " | Error: "
            + error
            + " | Strafe Speed: "
            + strafeSpeed);

    // Move the robot sideways without rotation
    drive.runRobotCentricChassisSpeeds(new ChassisSpeeds(0.0, strafeSpeed, 0.0));
  }

  @Override
  public boolean isFinished() {
    double tx = vision.getInputs().tx;
    boolean aligned = Math.abs(tx - desiredOffset) < tolerance;

    // System.out.println(
    //     "[StrafeToTarget] Checking if finished - tx: "
    //         + tx
    //         + " | Target: "
    //         + desiredOffset
    //         + " | Aligned: "
    //         + aligned);

    return aligned;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop(); // Stop the robot when finished
  }
}
