package frc.robot.subsystems.superstructure.Elevator;

import static frc.robot.subsystems.superstructure.Elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.util.Alert;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator {
  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Elevator/Gains/kP", gains.kP());
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber("Elevator/Gains/kI", gains.kI());
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Elevator/Gains/kD", gains.kD());
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Elevator/Gains/kS", gains.ffkS());
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Elevator/Gains/kV", gains.ffkV());
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Elevator/Gains/kA", gains.ffkA());
  private static final LoggedTunableNumber kG =
      new LoggedTunableNumber("Elevator/Gains/kG", gains.ffkG());
  private static final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber("Elevator/MaxVelocity", profileConstraints.maxVelocity);
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber("Elevator/MaxAcceleration", profileConstraints.maxAcceleration);

  // Profile constraints
  private static final Supplier<TrapezoidProfile.Constraints> profileConstraintsSupplier =
      () -> new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get());

  @RequiredArgsConstructor
  public enum Goal {
    // STOW(() -> 0.0),
    // GROUND_INTAKE(() -> 0.0),
    // SCORE_L1(() -> 0.0),
    // SCORE_L2(() -> 0.0),
    // SCORE_L3(() -> 0.0),
    // SCORE_L4(() -> 0.0);
    STOW(() -> 0.0),
    GROUND_INTAKE(() -> 0.005),
    DOWN_ELEVATOR(() -> 0.005),
    SCORE_L1(() -> 0.05),
    SCORE_L2(() -> 0.07),
    SCORE_L3(() -> 0.135),
    LOW_ALGAE(() -> 0.05),
    REMOVE_CORAL(() -> 0.16),
    UP_ALGAE(() -> 0.1);

    private final Supplier<Double> setpointSupplier;

    private double getMeters() {
      return setpointSupplier.get();
    }
  }

  @Getter @Setter @AutoLogOutput private Goal goal = Goal.STOW;
  private boolean characterizing = false;

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private TrapezoidProfile profile;
  private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();
  private double goalHeight;

  private ElevatorFeedforward ff;

  private final ElevatorVisualizer measuredVisualizer;
  private final ElevatorVisualizer setpointVisualizer;
  private final ElevatorVisualizer goalVisualizer;

  private BooleanSupplier disableSupplier = DriverStation::isDisabled;
  private boolean brakeModeEnabled = true;

  private final Alert elevatorMotorConnected =
      new Alert("Elevator motor disconnected!", Alert.AlertType.WARNING);

  public Elevator(ElevatorIO io) {
    this.io = io;
    io.setBrakeMode(true);

    profile = new TrapezoidProfile(profileConstraintsSupplier.get());
    io.setPID(kP.get(), kI.get(), kD.get());
    ff = new ElevatorFeedforward(kS.get(), kV.get(), kA.get());

    measuredVisualizer = new ElevatorVisualizer("Measured", Color.kBlack);
    setpointVisualizer = new ElevatorVisualizer("Setpoint", Color.kGreen);
    goalVisualizer = new ElevatorVisualizer("Goal", Color.kBlue);
  }

  public void periodic() {
    // Process inputs
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    // Set alerts
    elevatorMotorConnected.set(!inputs.leaderMotorConnected);

    // Update controllers dynamically
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> ff = new ElevatorFeedforward(kS.get(), kG.get(), kA.get()), kS, kG, kA);

    // Handle disable conditions
    if (disableSupplier.getAsBoolean()) {
      io.stop();
      setpointState = new TrapezoidProfile.State(inputs.positionMeters, 0.0);
      return;
    }

    if (goal == Goal.STOW) {
      goalHeight = Goal.STOW.getMeters();
    } else {
      goalHeight = goal.getMeters();
    }

    // Run closed loop
    goalHeight = goal.getMeters();
    setpointState =
        profile.calculate(
            Constants.loopPeriodSecs,
            setpointState,
            new TrapezoidProfile.State(
                MathUtil.clamp(goalHeight, minLengthMeters, maxLengthMeters), 0.0));

    if (EqualsUtil.epsilonEquals(goalHeight, minLengthMeters) || atGoal()) {
      Logger.recordOutput("Teste", true);
      io.stop();
    } else {
      Logger.recordOutput("Teste", false);
      io.runSetpoint(setpointState.position, ff.calculate(setpointState.velocity));
    }

    goalVisualizer.update(goalHeight);
    Logger.recordOutput("Elevator/GoalHeight", goalHeight);

    // Log setpoint and goal
    measuredVisualizer.update(inputs.positionMeters);
    setpointVisualizer.update(setpointState.position);
    Logger.recordOutput("Elevator/GoalHeight", goalHeight);
    Logger.recordOutput("Elevator/SetpointHeight", setpointState.position);
    Logger.recordOutput("Elevator/SetpointVelocity", setpointState.velocity);
  }

  public void stop() {
    io.stop();
  }

  @AutoLogOutput(key = "Superstructure/Elevator/AtGoal")
  public boolean atGoal() {
    return EqualsUtil.epsilonEquals(inputs.positionMeters, goalHeight, positionToleranceMeters);
  }

  public void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled == enabled) return;
    brakeModeEnabled = enabled;
    io.setBrakeMode(brakeModeEnabled);
  }

  public void runCharacterization(double amps) {
    characterizing = true;
    io.runCurrent(amps);
  }

  public double getCharacterizationVelocity() {
    return inputs.velocityMetersPerSec;
  }

  public void endCharacterization() {
    characterizing = false;
  }
}
