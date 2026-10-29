// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.Intake;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean intakeMotorConnected = true;
    public double positionRads = 0.0;
    public double absoluteEncoderPositionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVolts;
    public double supplyCurrentAmps;
    public double torqueCurrentAmps;
    public double tempCelcius;
    public boolean absoluteEncoderConnected = true;
  }

  default void updateInputs(IntakeIOInputs inputs) {}

  /** Run to setpoint angle in radians */
  default void runSetpoint(double setpointRads, double feedforward) {}

  /** Run motors at volts */
  default void runVolts(double volts) {}

  /** Run motors at current */
  default void runCurrent(double amps) {}

  /** Set brake mode enabled */
  default void setBrakeMode(boolean enabled) {}

  /** Set PID values */
  default void setPID(double p, double i, double d) {}

  /** Stops motors */
  default void stop() {}
}
