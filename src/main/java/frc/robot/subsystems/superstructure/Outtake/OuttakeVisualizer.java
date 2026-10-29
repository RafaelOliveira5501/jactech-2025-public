// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.Outtake;

import static frc.robot.subsystems.superstructure.Outtake.OuttakeConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** Add your docs here. */
public class OuttakeVisualizer {
  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismLigament2d outtake;
  private final String key;

  public OuttakeVisualizer(String key, Color color) {
    this.key = key;
    mechanism = new LoggedMechanism2d(3.0, 3.0, new Color8Bit(Color.kYellow));
    LoggedMechanismRoot2d root = mechanism.getRoot("pivot", 0.28, 0.5);
    outtake = new LoggedMechanismLigament2d("outtake", outtakeLenght, 180, 6, new Color8Bit(color));
    root.append(outtake);
  }

  /** Update outtake visualizer with current outtake angle */
  public void update(double angleRads) {
    // Log Mechanism2d
    outtake.setAngle(Rotation2d.fromRadians(angleRads));
    Logger.recordOutput("Outtake/Mechanism2d/" + key, mechanism);

    // Log 3D poses
    Pose3d pivot =
        new Pose3d(
            outtakeOrigin.getX(), 0.0, outtakeOrigin.getY(), new Rotation3d(0.0, -angleRads, 0.0));
    Logger.recordOutput("Outtake/Mechanism3d/" + key, pivot);
    Logger.recordOutput("Robot", new Pose2d());
  }
}
