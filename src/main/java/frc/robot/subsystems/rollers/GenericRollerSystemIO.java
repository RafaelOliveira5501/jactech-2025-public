package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface GenericRollerSystemIO {
  @AutoLog
  abstract class GenericRollerSystemIOInputs {
    public boolean connected = true;
    public double velocityRadsPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  default void updateInputs(GenericRollerSystemIOInputs inputs) {}

  /** Run feeder at volts */
  default void runVolts(double volts) {}

  /** Stop feeder */
  default void stop() {}
}
