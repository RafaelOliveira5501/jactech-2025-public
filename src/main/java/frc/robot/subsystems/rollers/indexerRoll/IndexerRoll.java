package frc.robot.subsystems.rollers.indexerRoll;

import frc.robot.subsystems.rollers.GenericRollerSystem;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class IndexerRoll extends GenericRollerSystem<IndexerRoll.Goal> {
  @RequiredArgsConstructor
  @Getter
  public enum Goal implements GenericRollerSystem.VoltageGoal {
    IDLING(() -> 0.0),
    INTAKE(new LoggedTunableNumber("Feeder/Intake Roller", -4.0)),
    OUTTAKE(new LoggedTunableNumber("Outtake/Outtake Roller", 4.0));

    private final DoubleSupplier voltageSupplier;
  }

  private IndexerRoll.Goal goal = IndexerRoll.Goal.IDLING;

  public IndexerRoll(IndexerRollIO io) {
    super("Outtake", io);
  }
}
