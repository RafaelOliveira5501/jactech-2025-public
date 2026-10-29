// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.rollers.intakeRoll;

import frc.robot.subsystems.rollers.GenericRollerSystem;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class IntakeRoll extends GenericRollerSystem<IntakeRoll.Goal> {
  @RequiredArgsConstructor
  @Getter
  public enum Goal implements GenericRollerSystem.VoltageGoal {
    IDLING(() -> 0.0),
    INTAKE(new LoggedTunableNumber("Intake/Intake Roller", 11.0)),
    OUTTAKE(new LoggedTunableNumber("Outtake/Outtake Roller", -4.0));

    private final DoubleSupplier voltageSupplier;
  }

  private IntakeRoll.Goal goal = IntakeRoll.Goal.IDLING;

  public IntakeRoll(IntakeRollIO io) {
    super("Outtake", io);
  }
}
