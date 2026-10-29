// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.Intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Add your docs here. */
public class IntakeConstants {
  public static final int leaderCanId = 22;
  public static final int followerCanId = 62;
  public static final int intakeEncoderID = 31;

  public static final int motionMagicSlot = 1;
  public static final int currentLimit = 40;

  // reduction is 12:62 18:60 12:65
  public static final double reduction = 75;
  public static final Rotation2d positionTolerance = Rotation2d.fromDegrees(3.0);

  public static final Rotation2d minAngle = Rotation2d.fromDegrees(-110);

  public static final Rotation2d maxAngle = Rotation2d.fromDegrees(0);
  public static final Gains gains = new Gains(75, 0.0, 0.0, 0.0, 0.0, 0.0, 1);

  public static final Translation2d intakeOrigin = new Translation2d(-0.238, 0.298);
  public static TrapezoidProfile.Constraints profileConstraints =
      new TrapezoidProfile.Constraints(1.75 * Math.PI, 15);

  public static final double intakeLenght = 0.55;

  public record Gains(
      double kP, double kI, double kD, double ffkS, double ffkV, double ffkA, double ffkG) {}
}
;
