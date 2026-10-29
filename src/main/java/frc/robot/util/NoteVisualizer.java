// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;
import java.util.stream.Stream;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class NoteVisualizer {
  private static final double shotSpeed = 9.0; // Meters per sec
  private static final double ejectSpeed = 2.0; // Meters per sec
  @Setter private static Supplier<Pose2d> robotPoseSupplier = Pose2d::new;
  @Setter private static Supplier<Rotation2d> armAngleSupplier = Rotation2d::new;
  private static final List<Translation2d> autoNotes = new ArrayList<>();
  @Setter private static boolean hasNote = false;

  /** Show all staged notes for alliance */
  public static void showAutoNotes() {
    if (autoNotes.isEmpty()) {
      Logger.recordOutput("NoteVisualizer/StagedNotes", new Pose3d[] {});
    }
    // Show auto notes
    Stream<Translation2d> presentNotes = autoNotes.stream().filter(Objects::nonNull);
    Logger.recordOutput(
        "NoteVisualizer/StagedNotes",
        presentNotes
            .map(
                translation ->
                    new Pose3d(
                        translation.getX(),
                        translation.getY(),
                        Units.inchesToMeters(1.0),
                        new Rotation3d()))
            .toArray(Pose3d[]::new));
  }

  public static void clearAutoNotes() {
    autoNotes.clear();
  }

  /**
   * Take note from staged note
   *
   * @param note Number of note starting with 0 - 2 being spike notes going from amp to source side
   *     <br>
   *     and 3 - 7 being centerline notes going from amp to source side.
   */
  public static void takeAutoNote(int note) {
    autoNotes.set(note, null);
    hasNote = true;
  }

  /** Shows the currently held note if there is one */

  /** Shoots note from middle of arm to speaker */
}
