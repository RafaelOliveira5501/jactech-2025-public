// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.rollers.outtakeRoll;

import frc.robot.subsystems.rollers.GenericRollerSystemIONEO;

public class OuttakeRollIONEO extends GenericRollerSystemIONEO implements OuttakeRollIO {
  private static final int id = 25;
  private static final String bus = "rio";
  private static final int currentLimitAmps = 40;
  private static final boolean invert = false;
  private static final boolean brake = false;
  private static final double reduction = 15;

  public OuttakeRollIONEO() {
    super(id, bus, currentLimitAmps, invert, brake, reduction);
  }
}
