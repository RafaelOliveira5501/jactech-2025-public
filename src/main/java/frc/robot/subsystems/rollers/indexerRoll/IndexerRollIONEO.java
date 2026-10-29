// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.rollers.indexerRoll;

import frc.robot.subsystems.rollers.IndexerRollerSystemIONEO;

public class IndexerRollIONEO extends IndexerRollerSystemIONEO implements IndexerRollIO {
  private static final int idLeader = 27;
  private static final int idFollower = 28;
  private static final int currentLimitAmps = 40;
  private static final boolean invert = true;
  private static final boolean brake = true;
  private static final double reduction = 1 / 25;

  public IndexerRollIONEO() {
    super(idLeader, idFollower, currentLimitAmps, invert, brake, reduction);
  }
}
