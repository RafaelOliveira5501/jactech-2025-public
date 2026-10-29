package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollersSensorsIO {
  @AutoLog
  class RollersSensorsIOInputs {
    boolean outtakeStaged = false;
    double sensorDistance;
  }

  default void updateInputs(RollersSensorsIOInputs inputs) {}
}
