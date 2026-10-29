package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.rollers.indexerRoll.IndexerRoll;
import frc.robot.subsystems.rollers.intakeRoll.IntakeRoll;
// import frc.robot.subsystems.rollers.indexer.Indexer;
// import frc.robot.subsystems.rollers.intakeRoll.IntakeRoll;
import frc.robot.subsystems.rollers.outtakeRoll.OuttakeRoll;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Rollers extends SubsystemBase {

  private OuttakeRoll outtakeRoll;
  private IndexerRoll indexerRoll;
  private IntakeRoll intakeRoll;
  private final RollersSensorsIO sensorsIO;
  private final RollersSensorsIOInputsAutoLogged sensorInputs =
      new RollersSensorsIOInputsAutoLogged();

  public enum Goal {
    IDLE,
    INTAKE,
    OUTTAKE,
    UP,
    DOWN,
    ALGAE
  }

  public enum GamepieceState {
    NONE,
    OUTTAKE_STAGED
  }

  @Getter private Goal goal = Goal.IDLE;
  @AutoLogOutput @Getter @Setter private GamepieceState gamepieceState = GamepieceState.NONE;
  private GamepieceState lastGamepieceState = GamepieceState.NONE;
  private Timer gamepieceStateTimer = new Timer();
  @Setter private BooleanSupplier outtakeActuatedSupplier = () -> false;

  public Rollers(
      OuttakeRoll outtakeRoll,
      IndexerRoll indexerRoll,
      IntakeRoll intakeRoll,
      RollersSensorsIO sensorsIO) {

    this.outtakeRoll = outtakeRoll;
    this.indexerRoll = indexerRoll;
    this.intakeRoll = intakeRoll;
    this.sensorsIO = sensorsIO;

    setDefaultCommand(setGoalCommand(Goal.IDLE));
    gamepieceStateTimer.start();
  }

  @Override
  public void periodic() {
    sensorsIO.updateInputs(sensorInputs);
    Logger.processInputs("RollersSensors", sensorInputs);

    if (DriverStation.isDisabled()) {
      goal = Goal.IDLE;
    }

    if (sensorInputs.outtakeStaged) {
      gamepieceState = GamepieceState.OUTTAKE_STAGED;
    } else {
      gamepieceState = GamepieceState.NONE;
    }

    if (gamepieceState != lastGamepieceState) {
      gamepieceStateTimer.reset();
    }
    lastGamepieceState = gamepieceState;

    outtakeRoll.setGoal(OuttakeRoll.Goal.IDLING);
    indexerRoll.setGoal(IndexerRoll.Goal.IDLING);
    intakeRoll.setGoal(IntakeRoll.Goal.IDLING);

    switch (goal) {
      case IDLE -> {
        outtakeRoll.setGoal(OuttakeRoll.Goal.IDLING);
        indexerRoll.setGoal(IndexerRoll.Goal.IDLING);
        intakeRoll.setGoal(IntakeRoll.Goal.IDLING);
      }

      case INTAKE -> {
        if (gamepieceState == GamepieceState.OUTTAKE_STAGED) {
          outtakeRoll.setGoal(OuttakeRoll.Goal.IDLING);
          indexerRoll.setGoal(IndexerRoll.Goal.IDLING);
          intakeRoll.setGoal(IntakeRoll.Goal.IDLING);
        } else {
          outtakeRoll.setGoal(OuttakeRoll.Goal.INTAKE);
          indexerRoll.setGoal(IndexerRoll.Goal.INTAKE);
          intakeRoll.setGoal(IntakeRoll.Goal.INTAKE);
        }
      }

      case UP -> {
        outtakeRoll.setGoal(OuttakeRoll.Goal.UP);
      }
      case DOWN -> {
        outtakeRoll.setGoal(OuttakeRoll.Goal.DOWN);
      }

      case OUTTAKE -> {
        outtakeRoll.setGoal(OuttakeRoll.Goal.OUTTAKE);
        indexerRoll.setGoal(IndexerRoll.Goal.OUTTAKE);
        intakeRoll.setGoal(IntakeRoll.Goal.OUTTAKE);
      }
      case ALGAE -> {
        outtakeRoll.setGoal(OuttakeRoll.Goal.ALGAE);
      }

      default -> throw new IllegalArgumentException("Unexpected value: " + goal);
    }

    outtakeRoll.periodic();
    indexerRoll.periodic();
    intakeRoll.periodic();
  }

  public Command setGoalCommand(Goal goal) {
    return startEnd(() -> this.goal = goal, () -> this.goal = Goal.IDLE)
        .withName("Rollers " + goal);
  }

  public boolean getGamePieceStage() {
    return gamepieceState == GamepieceState.OUTTAKE_STAGED;
  }
}
