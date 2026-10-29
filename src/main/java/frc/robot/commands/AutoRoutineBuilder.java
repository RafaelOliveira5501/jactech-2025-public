package frc.robot.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.VisionLimelight;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import java.util.Optional;
import java.util.function.Supplier;

public class AutoRoutineBuilder {
  private final AutoFactory autoFactory;
  private final Drive drive;
  private VisionLimelight limelightFront;
  private Rollers rollers;
  private Superstructure superstructure;

  public AutoRoutineBuilder(
      AutoFactory autoFactory,
      Drive drive,
      VisionLimelight limelightFront,
      Rollers rollers,
      Superstructure superstructure) {
    this.autoFactory = autoFactory;
    this.drive = drive;
    this.limelightFront = limelightFront;
    this.rollers = rollers;
    this.superstructure = superstructure;
  }

  public AutoRoutine pickupAndScoreAuto() {
    AutoRoutine routine = autoFactory.newRoutine("PickupAndScore");

    // Create alignment commands
    Command moveToReef0 =
        new AutoAlignment(
            drive,
            () -> AllianceFlipUtil.apply(FieldConstants.Reef.reefScoringPosition[0].get()),
            () -> AllianceFlipUtil.apply(FieldConstants.Reef.reefScoringPosition[0].get()),
            new Pose2d(0.8, 0.8, new Rotation2d(15)),
            1.0);
    Command moveToReef1 =
        new AutoAlignment(
            drive,
            () -> AllianceFlipUtil.apply(FieldConstants.Reef.reefScoringPosition[1].get()),
            () -> AllianceFlipUtil.apply(FieldConstants.Reef.reefScoringPosition[1].get()),
            new Pose2d(0.8, 0.8, new Rotation2d(15)),
            1.0);
    Command moveToReef2 =
        new AutoAlignment(
            drive,
            FieldConstants.Reef.reefScoringPosition[2],
            FieldConstants.Reef.reefScoringPosition[2],
            new Pose2d(0.8, 0.8, new Rotation2d(15)),
            1.0);
    Command moveToReef3 =
        new AutoAlignment(
            drive,
            FieldConstants.Reef.reefScoringPosition[3],
            FieldConstants.Reef.reefScoringPosition[3],
            new Pose2d(0.8, 0.8, new Rotation2d(15)),
            1.0);
    Command moveToReef4 =
        new AutoAlignment(
            drive,
            FieldConstants.Reef.reefScoringPosition[5],
            FieldConstants.Reef.reefScoringPosition[5],
            new Pose2d(0.8, 0.8, new Rotation2d(15)),
            1.0);
    Command moveToReef5 =
        new AutoAlignment(
            drive,
            FieldConstants.Reef.reefScoringPosition[3],
            FieldConstants.Reef.reefScoringPosition[3],
            new Pose2d(0.8, 0.8, new Rotation2d(15)),
            1.0);

    Command alignBranchRight =
        Commands.runOnce(
            () -> {
              Optional<Supplier<Pose2d>[]> optionalPoseSuppliers =
                  limelightFront.getTargetPoseForClosestTag();

              if (optionalPoseSuppliers.isPresent()) {
                Supplier<Pose2d>[] poses = optionalPoseSuppliers.get();
                Supplier<Pose2d> targetPose = poses[0];
                Supplier<Pose2d> rightPose = poses[2];

                System.out.println("[MoveToTagPose] Moving to: " + targetPose.get());

                // Create AutoAlignment with the correct poses
                AutoAlignment autoAlignment =
                    new AutoAlignment(
                        drive,
                        targetPose,
                        rightPose,
                        new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(3)),
                        0.5);

                // Schedule the pathfinding + strafe command
                autoAlignment.schedule();
                ;

              } else {
                System.out.println(
                    "[MoveToTagPose] No valid target pose found. Skipping movement.");
              }
            });

    Command alignBranchLeft =
        Commands.runOnce(
            () -> {
              Optional<Supplier<Pose2d>[]> optionalPoseSuppliers =
                  limelightFront.getTargetPoseForClosestTag();

              if (optionalPoseSuppliers.isPresent()) {
                Supplier<Pose2d>[] poses = optionalPoseSuppliers.get();
                Supplier<Pose2d> targetPose = poses[0];
                Supplier<Pose2d> leftPose = poses[1];

                System.out.println("[MoveToTagPose] Moving to: " + targetPose.get());

                // Create AutoAlignment with the correct poses
                AutoAlignment autoAlignment =
                    new AutoAlignment(
                        drive,
                        targetPose,
                        leftPose,
                        new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(3)),
                        0.5);

                // Schedule the pathfinding + strafe command
                autoAlignment.schedule();
                ;

              } else {
                System.out.println(
                    "[MoveToTagPose] No valid target pose found. Skipping movement.");
              }
            });

    Command fullCycle = Commands.sequence(moveToReef2.andThen(l3()));

    // Register command sequence to run when the routine becomes active
    routine.active().onTrue(fullCycle);

    return routine;
  }

  private Command l3() {
    return new SequentialCommandGroup(
        superstructure.setGoalCommand(Superstructure.Goal.L3).withName("Score l3"),
        new WaitCommand(1),
        rollers.setGoalCommand(Rollers.Goal.OUTTAKE).withName("Outtake Roller"));
  }
}
