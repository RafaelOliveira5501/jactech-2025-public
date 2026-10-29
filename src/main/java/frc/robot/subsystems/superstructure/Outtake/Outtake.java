// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.Outtake;

import static frc.robot.subsystems.superstructure.Outtake.OuttakeConstants.*;

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

public class Outtake {
  /** Add your docs here. */
  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Outtake/Gains/kP", gains.kP());

  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber("Outtake/Gains/kI", gains.kI());
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Outtake/Gains/kD", gains.kD());
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Outtake/Gains/kS", gains.ffkS());
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Outtake/Gains/kV", gains.ffkV());
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Outtake/Gains/kA", gains.ffkA());
  private static final LoggedTunableNumber kG =
      new LoggedTunableNumber("Outtake/Gains/kG", gains.ffkG());
  private static final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber("Outtake/Velocity", profileConstraints.maxVelocity);
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber("Outtake/Acceleration", profileConstraints.maxAcceleration);
  private static final LoggedTunableNumber smoothVelocity =
      new LoggedTunableNumber("Outtake/SmoothVelocity", profileConstraints.maxVelocity * 0.75);
  private static final LoggedTunableNumber smoothAcceleration =
      new LoggedTunableNumber(
          "Outtake/SmoothAcceleration", profileConstraints.maxAcceleration * 0.5);
  private static final LoggedTunableNumber prepareClimbVelocity =
      new LoggedTunableNumber("Outtake/PrepareClimbVelocity", 1.5);
  private static final LoggedTunableNumber prepareClimbAcceleration =
      new LoggedTunableNumber("Outtake/PrepareClimbAcceleration", 2.5);
  private static final LoggedTunableNumber lowerLimitDegrees =
      new LoggedTunableNumber("Outtake/LowerLimitDegrees", minAngle.getDegrees());
  private static final LoggedTunableNumber upperLimitDegrees =
      new LoggedTunableNumber("Outtake/UpperLimitDegrees", maxAngle.getDegrees());
  private static final LoggedTunableNumber partialStowUpperLimitDegrees =
      new LoggedTunableNumber("Outtake/PartialStowUpperLimitDegrees", 30.0);

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
    STOW((new LoggedTunableNumber("Outtake/STOW", 150))),
    GROUND_INTAKE(new LoggedTunableNumber("Outtake/GROUND_INTAKE", 150)), // 152
    DOWN_ELEVATOR(new LoggedTunableNumber("Outtake/GROUND_INTAKE", -45)),
    LOW_ALGAE(new LoggedTunableNumber("Outtake/GROUND_INTAKE", 30)),
    UP_ALGAE(new LoggedTunableNumber("Outtake/GROUND_INTAKE", 0)),
    SCORE_L1(new LoggedTunableNumber("Outtake/SCORE_L1", -45)),
    SCORE_L2(new LoggedTunableNumber("Outtake/SCORE_L2", -45)),
    SCORE_L3(new LoggedTunableNumber("Outtake/SCORE_L3", -45));

    private final DoubleSupplier armSetpointSupplier;

    private double getRads() {
      return Units.degreesToRadians(armSetpointSupplier.getAsDouble());
    }
  }

  @AutoLogOutput @Getter @Setter private Goal goal = Goal.STOW;
  private boolean characterizing = false;

  private final OuttakeIO io;
  private final OuttakeIOInputsAutoLogged inputs = new OuttakeIOInputsAutoLogged();

  @AutoLogOutput @Setter private double currentCompensation = 0.0;
  private TrapezoidProfile.Constraints currentConstraints = maxProfileConstraints.get();
  private TrapezoidProfile profile;
  private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();

  private double goalAngle;
  private ArmFeedforward ff;

  private final OuttakeVisualizer measuredVisualizer;
  private final OuttakeVisualizer setpointVisualizer;
  private final OuttakeVisualizer goalVisualizer;

  private BooleanSupplier disableSupplier = DriverStation::isDisabled;
  private BooleanSupplier coastSupplier = () -> false;
  private BooleanSupplier halfStowSupplier = () -> true;
  private boolean brakeModeEnabled = true;

  private boolean wasNotAuto = false;

  private final Alert outtakeMotorConnected =
      new Alert("Outtake motor disconnected!", Alert.AlertType.WARNING);

  private final Alert absoluteEncoderDisconnected =
      new Alert("Outtake absolute encoder disconnected!", Alert.AlertType.WARNING);

  public Outtake(OuttakeIO io) {
    this.io = io;
    io.setBrakeMode(true);

    profile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
    io.setPID(kP.get(), kI.get(), kD.get());
    ff = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

    // Set up visualizers
    measuredVisualizer = new OuttakeVisualizer("Measured", Color.kBlack);
    setpointVisualizer = new OuttakeVisualizer("Setpoint", Color.kGreen);
    goalVisualizer = new OuttakeVisualizer("Goal", Color.kOrange);
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
    Logger.processInputs("Outtake", inputs);

    // Set alerts
    outtakeMotorConnected.set(!inputs.outtakeMotorConnected);
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

    if (EqualsUtil.epsilonEquals(goalAngle, minAngle.getRadians())
        || atGoal()
        || goal == Goal.STOW) {
      io.stop();
      Logger.recordOutput("Teste", true);
    } else {
      Logger.recordOutput("Teste", false);
      io.runSetpoint(
          setpointState.position, ff.calculate(setpointState.position, setpointState.velocity));
    }

    goalVisualizer.update(goalAngle);
    Logger.recordOutput("Outtake/GoalAngle", goalAngle);

    // Logs
    measuredVisualizer.update(inputs.positionRads);
    setpointVisualizer.update(setpointState.position);
    Logger.recordOutput(
        "Outtake/EpsilonEquals", EqualsUtil.epsilonEquals(goalAngle, minAngle.getRadians()));
    Logger.recordOutput("Outtake/SetpointAngle", setpointState.position);
    Logger.recordOutput("Outtake/PositionRads", inputs.positionRads);
    Logger.recordOutput("Outtake/SetpointVelocity", setpointState.velocity);
    Logger.recordOutput("Superstructure/Outtake/Goal", goal);
  }

  public void stop() {
    io.stop();
  }

  @AutoLogOutput(key = "Superstructure/Outtake/AtGoal")
  public boolean atGoal() {
    return EqualsUtil.epsilonEquals(inputs.positionRads, goalAngle, 0.25);
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
