// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.Outtake;

import static frc.robot.subsystems.superstructure.Outtake.OuttakeConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

/** Add your docs here. */
public class OuttakeIOSim implements OuttakeIO {
  private static final double autoStartAngle = Units.degreesToRadians(0.0);

  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          reduction,
          1.06328,
          outtakeLenght,
          minAngle.getRadians(),
          maxAngle.getRadians(),
          false,
          Units.degreesToRadians(0.0));

  private final PIDController controller;
  private double appliedVoltage = 0.0;
  private double positionOffset = 0.0;

  private boolean controllerNeedsReset = false;
  private boolean closedLoop = true;
  private boolean wasNotAuto = true;

  public OuttakeIOSim() {
    controller = new PIDController(1.0, 0.0, 0.0);
    sim.setState(0.0, 0.0);
    setPosition(0.0);
  }

  @Override
  public void updateInputs(OuttakeIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      controllerNeedsReset = true;
    }

    if (wasNotAuto && DriverStation.isAutonomousEnabled()) {
      sim.setState(autoStartAngle, 0.0);
      wasNotAuto = false;
    }

    wasNotAuto = !DriverStation.isAutonomousEnabled();
    sim.update(Constants.loopPeriodSecs);

    inputs.positionRads = sim.getAngleRads() + positionOffset;
    inputs.velocityRadsPerSec = sim.getVelocityRadPerSec();
    inputs.appliedVolts = appliedVoltage;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = sim.getCurrentDrawAmps();
    inputs.tempCelcius = 0.0;

    sim.setInputVoltage(0.0);
  }

  @Override
  public void runSetpoint(double setpointRads, double feedforward) {
    if (!closedLoop) {
      controllerNeedsReset = true;
      closedLoop = true;
    }
    if (controllerNeedsReset) {
      controller.reset();
      controllerNeedsReset = false;
    }
    runVolts(controller.calculate(sim.getAngleRads(), setpointRads + positionOffset) + feedforward);
  }

  @Override
  public void runVolts(double volts) {
    closedLoop = false;
    appliedVoltage = MathUtil.clamp(volts, -12, 12);
    sim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void setPID(double p, double i, double d) {
    controller.setPID(p, i, d);
  }

  @Override
  public void stop() {
    appliedVoltage = 0.0;
    sim.setInputVoltage(appliedVoltage);
  }

  private void setPosition(double position) {
    positionOffset = position - sim.getAngleRads();
  }
}
