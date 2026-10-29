// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.rollers.intakeRoll;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.rollers.GenericRollerSystemIOSim;

public class IntakeRollIOSim extends GenericRollerSystemIOSim implements IntakeRollIO {
  private static final DCMotor motorModel = DCMotor.getNEO(1);
  private static final double reduction = 18.0 / 12.0;
  private static final double moi = 0.001;

  public IntakeRollIOSim() {
    super(motorModel, reduction, moi);
  }
}
