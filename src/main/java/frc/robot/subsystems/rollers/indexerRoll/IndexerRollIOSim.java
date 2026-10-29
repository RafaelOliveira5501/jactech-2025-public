// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.rollers.indexerRoll;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.rollers.GenericRollerSystemIOSim;

public class IndexerRollIOSim extends GenericRollerSystemIOSim implements IndexerRollIO {
  private static final DCMotor motorModel = DCMotor.getNEO(2);
  private static final double reduction = 1.0 / 25.0;
  private static final double moi = 0.001;

  public IndexerRollIOSim() {
    super(motorModel, reduction, moi);
  }
}
