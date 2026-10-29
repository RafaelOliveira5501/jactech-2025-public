// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.Intake;

import static frc.robot.subsystems.superstructure.Intake.IntakeConstants.*;

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
public class IntakeVisualizer {
  private final LoggedMechanism2d mechanism;
  private final LoggedMechanismLigament2d intake;
  private final String key;

  public IntakeVisualizer(String key, Color color) {
    this.key = key;
    mechanism = new LoggedMechanism2d(3.0, 3.0, new Color8Bit(Color.kYellow));
    LoggedMechanismRoot2d root = mechanism.getRoot("pivot", -0.375, 0.22);
    intake = new LoggedMechanismLigament2d("intake", intakeLenght, 0.0, 6, new Color8Bit(color));
    root.append(intake);
  }

  /** Update intake visualizer with current intake angle */
  public void update(double angleRads) {
    // Log Mechanism2d
    intake.setAngle(Rotation2d.fromRadians(angleRads));
    Logger.recordOutput("Intake/Mechanism2d/" + key, mechanism);

    // Log 3D poses
    Pose3d pivot =
        new Pose3d(
            intakeOrigin.getX(), 0.0, intakeOrigin.getY(), new Rotation3d(0.0, -angleRads, 0.0));
    Logger.recordOutput("Intake/Mechanism3d/" + key, pivot);
  }
}
