// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.Intake;

import static frc.robot.subsystems.superstructure.Intake.IntakeConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.util.Alert;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake {
  /** Add your docs here. */
  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Intake/Gains/kP", gains.kP());

  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber("Intake/Gains/kI", gains.kI());
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Intake/Gains/kD", gains.kD());
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Intake/Gains/kS", gains.ffkS());
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Intake/Gains/kV", gains.ffkV());
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Intake/Gains/kA", gains.ffkA());
  private static final LoggedTunableNumber kG =
      new LoggedTunableNumber("Intake/Gains/kG", gains.ffkG());
  private static final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber("Intake/Velocity", profileConstraints.maxVelocity);
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber("Intake/Acceleration", profileConstraints.maxAcceleration);
  private static final LoggedTunableNumber smoothVelocity =
      new LoggedTunableNumber("Intake/SmoothVelocity", profileConstraints.maxVelocity * 0.75);
  private static final LoggedTunableNumber smoothAcceleration =
      new LoggedTunableNumber(
          "Intake/SmoothAcceleration", profileConstraints.maxAcceleration * 0.5);
  private static final LoggedTunableNumber prepareClimbVelocity =
      new LoggedTunableNumber("Intake/PrepareClimbVelocity", 1.5);
  private static final LoggedTunableNumber prepareClimbAcceleration =
      new LoggedTunableNumber("Intake/PrepareClimbAcceleration", 2.5);
  private static final LoggedTunableNumber lowerLimitDegrees =
      new LoggedTunableNumber("Intake/LowerLimitDegrees", minAngle.getDegrees());
  private static final LoggedTunableNumber upperLimitDegrees =
      new LoggedTunableNumber("Intake/UpperLimitDegrees", maxAngle.getDegrees());
  private static final LoggedTunableNumber partialStowUpperLimitDegrees =
      new LoggedTunableNumber("Intake/PartialStowUpperLimitDegrees", 30.0);

  // Profile constraints
  public static final Supplier<TrapezoidProfile.Constraints> maxProfileConstraints =
      () -> new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get());
  public static final Supplier<TrapezoidProfile.Constraints> smoothProfileConstraints =
      () -> new TrapezoidProfile.Constraints(smoothVelocity.get(), smoothAcceleration.get());
  public static final Supplier<TrapezoidProfile.Constraints> prepareClimbProfileConstraints =
      () ->
          new TrapezoidProfile.Constraints(
              prepareClimbVelocity.get(), prepareClimbAcceleration.get());

  @RequiredArgsConstructor
  public enum Goal {
    STOW((new LoggedTunableNumber("Intake/STOW", -105))),
    GROUND_INTAKE(new LoggedTunableNumber("Intake/GROUND_INTAKE", -2.5));

    private final DoubleSupplier armSetpointSupplier;

    private double getRads() {
      return Units.degreesToRadians(armSetpointSupplier.getAsDouble());
    }
  }

  @AutoLogOutput @Getter @Setter private Goal goal = Goal.STOW;
  private boolean characterizing = false;

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  @AutoLogOutput @Setter private double currentCompensation = 0.0;
  private TrapezoidProfile.Constraints currentConstraints = maxProfileConstraints.get();
  private TrapezoidProfile profile;
  private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();

  private double goalAngle;
  private ArmFeedforward ff;

  private final IntakeVisualizer measuredVisualizer;
  private final IntakeVisualizer setpointVisualizer;
  private final IntakeVisualizer goalVisualizer;

  private BooleanSupplier disableSupplier = DriverStation::isDisabled;
  private BooleanSupplier coastSupplier = () -> false;
  private BooleanSupplier halfStowSupplier = () -> true;
  private boolean brakeModeEnabled = true;

  private boolean wasNotAuto = false;

  private final Alert intakeMotorConnected =
      new Alert("Intake motor disconnected!", Alert.AlertType.WARNING);

  private final Alert absoluteEncoderDisconnected =
      new Alert("Intake absolute encoder disconnected!", Alert.AlertType.WARNING);

  public Intake(IntakeIO io) {
    this.io = io;
    io.setBrakeMode(true);

    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
    io.setPID(kP.get(), kI.get(), kD.get());
    ff = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

    // Set up visualizers
    measuredVisualizer = new IntakeVisualizer("Measured", Color.kBlack);
    setpointVisualizer = new IntakeVisualizer("Setpoint", Color.kGreen);
    goalVisualizer = new IntakeVisualizer("Goal", Color.kOrange);
  }

  public void setOverrides(
      BooleanSupplier disableOverride,
      BooleanSupplier coastOverride,
      BooleanSupplier halfStowOverride) {
    disableSupplier = () -> disableOverride.getAsBoolean() || DriverStation.isDisabled();
    coastSupplier = coastOverride;
    halfStowSupplier = halfStowOverride;
  }

  private double getStowAngle() {
    return MathUtil.clamp(
        setpointState.position,
        minAngle.getRadians(),
        Units.degreesToRadians(partialStowUpperLimitDegrees.get()));
  }

  public void periodic() {
    // Process inputs
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    // Set alerts
    intakeMotorConnected.set(!inputs.intakeMotorConnected);
    absoluteEncoderDisconnected.set(!inputs.absoluteEncoderConnected);

    // Update controllers
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> ff = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get()),
        kS,
        kG,
        kV,
        kA);

    if (disableSupplier.getAsBoolean()
        || (Constants.getMode() == Constants.Mode.SIM
            && DriverStation.isAutonomousEnabled()
            && wasNotAuto)) {
      io.stop();
      // Reset profile when disabled
      setpointState = new TrapezoidProfile.State(inputs.positionRads, 0);
    }

    wasNotAuto = !DriverStation.isAutonomousEnabled();
    setBrakeMode(!coastSupplier.getAsBoolean());

    // Update goalAngle based on the current goal
    if (goal == Goal.STOW) {
      goalAngle = Goal.STOW.getRads();
    } else {
      goalAngle = goal.getRads();
    }

    double clampedGoal =
        MathUtil.clamp(
            goalAngle,
            Units.degreesToRadians(lowerLimitDegrees.get()),
            Units.degreesToRadians(upperLimitDegrees.get()));

    setpointState =
        profile.calculate(
            Constants.loopPeriodSecs,
            setpointState,
            new TrapezoidProfile.State(
                MathUtil.clamp(
                    clampedGoal,
                    Units.degreesToRadians(lowerLimitDegrees.get()),
                    Units.degreesToRadians(upperLimitDegrees.get())),
                0.0));

    if (EqualsUtil.epsilonEquals(goalAngle, minAngle.getRadians()) || atGoal()) {
      io.stop();
      Logger.recordOutput("Teste", true);
    } else {
      Logger.recordOutput("Teste", false);
      io.runSetpoint(
          setpointState.position, ff.calculate(setpointState.position, setpointState.velocity));
    }

    goalVisualizer.update(goalAngle);
    Logger.recordOutput("Intake/GoalAngle", goalAngle);

    // Logs
    measuredVisualizer.update(inputs.positionRads);
    setpointVisualizer.update(setpointState.position);
    Logger.recordOutput("Intake/SetpointAngle", setpointState.position);
    Logger.recordOutput("Intake/PositionRads", inputs.positionRads);
    Logger.recordOutput("Intake/SetpointVelocity", setpointState.velocity);
    Logger.recordOutput("Superstructure/Intake/Goal", goal);
  }

  public void stop() {
    io.stop();
  }

  @AutoLogOutput(key = "Superstructure/Intake/AtGoal")
  public boolean atGoal() {
    return EqualsUtil.epsilonEquals(inputs.positionRads, goalAngle, 0.1);
  }

  public void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled) return;
    brakeModeEnabled = enabled;
    io.setBrakeMode(brakeModeEnabled);
  }

  public void setProfileConstraints(TrapezoidProfile.Constraints constraints) {
    if (EqualsUtil.epsilonEquals(currentConstraints.maxVelocity, constraints.maxVelocity)
        && EqualsUtil.epsilonEquals(currentConstraints.maxAcceleration, constraints.maxVelocity))
      return;
    currentConstraints = constraints;
    profile = new TrapezoidProfile(currentConstraints);
  }

  public void runCharacterization(double amps) {
    characterizing = true;
    io.runCurrent(amps);
  }

  public double getCharacterizationVelocity() {
    return inputs.velocityRadsPerSec;
  }

  public void endCharacterization() {
    characterizing = false;
  }
}
