// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.rollers.outtakeRoll;

import frc.robot.subsystems.rollers.GenericRollerSystem;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class OuttakeRoll extends GenericRollerSystem<OuttakeRoll.Goal> {
  @RequiredArgsConstructor
  @Getter
  public enum Goal implements GenericRollerSystem.VoltageGoal {
    IDLING(() -> 0.0),
    INTAKE(new LoggedTunableNumber("Feeder/Intake Roller", 3)),
    OUTTAKE(new LoggedTunableNumber("Outtake/Outtake Roller", -2.0)),
    UP(new LoggedTunableNumber("Outtake/Place", 0.5)),
    DOWN(new LoggedTunableNumber("Outtake/Place", -2.0)),
    ALGAE(new LoggedTunableNumber("Outtake/Place", -12.0));

    private final DoubleSupplier voltageSupplier;
  }

  private OuttakeRoll.Goal goal = OuttakeRoll.Goal.IDLING;

  public OuttakeRoll(OuttakeRollIO io) {
    super("Outtake", io);
  }
}
