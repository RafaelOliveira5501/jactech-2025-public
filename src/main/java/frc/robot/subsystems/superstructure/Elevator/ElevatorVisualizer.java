package frc.robot.subsystems.superstructure.Elevator;

import static frc.robot.subsystems.superstructure.Elevator.ElevatorConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** Visualizer for the Elevator mechanism. */
public class ElevatorVisualizer {
  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismLigament2d elevator;
  private final String key;

  public ElevatorVisualizer(String key, Color color) {
    this.key = key;

    // Define o mecanismo 2D
    mechanism = new LoggedMechanism2d(1.0, maxLengthMeters, new Color8Bit(Color.kWhite));
    LoggedMechanismRoot2d root = mechanism.getRoot("base", 0.5, 0.0);
    elevator = new LoggedMechanismLigament2d("elevator", 0.0, 90.0, 6, new Color8Bit(color));
    root.append(elevator);
  }

  /**
   * Atualiza o visualizador do elevador com a posição atual.
   *
   * @param heightMeters Altura atual do elevador (em metros).
   */
  public void update(double heightMeters) {
    // Atualiza o mecanismo 2D
    elevator.setLength(heightMeters);
    Logger.recordOutput("Elevator/Mechanism2d/" + key, mechanism);

    // Atualiza o mecanismo 3D
    Pose3d basePose =
        new Pose3d(elevatorOrigin.getX(), elevatorOrigin.getY(), 0.0, new Rotation3d());
    Transform3d elevatorTransform =
        new Transform3d(
            new edu.wpi.first.math.geometry.Translation3d(0.0, 0.0, heightMeters),
            new Rotation3d());
    Pose3d elevatorPose = basePose.transformBy(elevatorTransform);
    Logger.recordOutput("Elevator/Mechanism3d/" + key, elevatorPose);
  }
}
