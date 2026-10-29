// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutoAlignment;
import frc.robot.commands.AutoRoutineBuilder;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.RollersSensorsIO;
import frc.robot.subsystems.rollers.RollersSensorsIOCompbot;
import frc.robot.subsystems.rollers.indexerRoll.IndexerRoll;
import frc.robot.subsystems.rollers.indexerRoll.IndexerRollIO;
import frc.robot.subsystems.rollers.indexerRoll.IndexerRollIONEO;
import frc.robot.subsystems.rollers.indexerRoll.IndexerRollIOSim;
import frc.robot.subsystems.rollers.intakeRoll.IntakeRoll;
import frc.robot.subsystems.rollers.intakeRoll.IntakeRollIO;
import frc.robot.subsystems.rollers.intakeRoll.IntakeRollIONEO;
import frc.robot.subsystems.rollers.intakeRoll.IntakeRollIOSim;
import frc.robot.subsystems.rollers.outtakeRoll.OuttakeRoll;
import frc.robot.subsystems.rollers.outtakeRoll.OuttakeRollIO;
import frc.robot.subsystems.rollers.outtakeRoll.OuttakeRollIONEO;
import frc.robot.subsystems.rollers.outtakeRoll.OuttakeRollIOSim;
import frc.robot.subsystems.superstructure.Elevator.Elevator;
import frc.robot.subsystems.superstructure.Elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.Elevator.ElevatorIONeo;
import frc.robot.subsystems.superstructure.Elevator.ElevatorIOSim;
import frc.robot.subsystems.superstructure.Intake.Intake;
import frc.robot.subsystems.superstructure.Intake.IntakeIO;
import frc.robot.subsystems.superstructure.Intake.IntakeIOKraken;
import frc.robot.subsystems.superstructure.Intake.IntakeIOSim;
import frc.robot.subsystems.superstructure.Outtake.Outtake;
import frc.robot.subsystems.superstructure.Outtake.OuttakeIO;
import frc.robot.subsystems.superstructure.Outtake.OuttakeIOKraken;
import frc.robot.subsystems.superstructure.Outtake.OuttakeIOSim;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOLimelightSim;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.subsystems.vision.VisionLimelight;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private VisionLimelight limelightFront;
  private Superstructure superstructure;
  private Rollers rollers;
  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.WARNING);
  private final Alert operatorDisconnected =
      new Alert("Operator controller disconnected (port 1).", AlertType.WARNING);

  // Dashboard inputs
  private LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Chooser");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Superstructire
    Elevator elevator = null;
    Outtake outtake = null;
    Intake intake = null;

    // Rollers
    OuttakeRoll outtakeRoll = null;
    IndexerRoll indexerRoll = null;
    IntakeRoll intakeRoll = null;
    RollersSensorsIO rollersSensorsIO = null;

    List<Transform3d> robotToCameras = null;
    switch (Constants.getMode()) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));

        // Real robot: initialize VisionIOPhotonVision for each camera

        limelightFront = new VisionLimelight(new VisionIOLimelight("limelight"));

        new Vision(
            new VisionIOPhotonVision(drive), () -> new Pose3d(drive.getPose())
            // ,limelightFront // Pass robot pose supplier
            );

        elevator = new Elevator(new ElevatorIONeo());
        outtake = new Outtake(new OuttakeIOKraken());
        intake = new Intake(new IntakeIOKraken());
        outtakeRoll = new OuttakeRoll(new OuttakeRollIONEO());
        indexerRoll = new IndexerRoll(new IndexerRollIONEO());
        intakeRoll = new IntakeRoll(new IntakeRollIONEO());
        rollersSensorsIO = new RollersSensorsIOCompbot();
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());

        // Initialize the Vision subsystem with VisionIOSim
        robotToCameras =
            List.of(
                new Transform3d(
                    new Translation3d(0.3, 0.3, 0.28),
                    new Rotation3d(
                        0.0, Math.toRadians(-10), Math.toRadians(90))), // Front Left camera
                new Transform3d(
                    new Translation3d(0.3, -0.3, 0.28),
                    new Rotation3d(
                        0.0, Math.toRadians(-10), Math.toRadians(0))), // Front Right camera
                new Transform3d(
                    new Translation3d(-0.3, 0.3, 0.28),
                    new Rotation3d(
                        0.0, Math.toRadians(-10), Math.toRadians(180))), // Back Left camera
                new Transform3d(
                    new Translation3d(-0.3, -0.3, 0.28),
                    new Rotation3d(
                        0.0, Math.toRadians(-10), Math.toRadians(270))) // Back Right camera
                );

        limelightFront =
            new VisionLimelight(
                new VisionIOLimelightSim(
                    drive,
                    VisionConstants.APRILTAG_FIELD_LAYOUT,
                    VisionConstants.LIMELIGHT_TRANSFORM));

        new Vision(
            new VisionIOSim(
                VisionConstants.APRILTAG_FIELD_LAYOUT,
                drive::getPose, // Pose supplier for simulation
                robotToCameras, // All camera transforms
                drive),
            () -> new Pose3d(drive.getPose())
            // ,
            // limelightFront // Pass robot pose supplier
            );

        elevator = new Elevator(new ElevatorIOSim());
        outtake = new Outtake(new OuttakeIOSim());
        intake = new Intake(new IntakeIOSim());
        outtakeRoll = new OuttakeRoll(new OuttakeRollIOSim());
        indexerRoll = new IndexerRoll(new IndexerRollIOSim());
        intakeRoll = new IntakeRoll(new IntakeRollIOSim());
        rollersSensorsIO = new RollersSensorsIO() {};
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        if (outtake == null) {
          outtake = new Outtake(new OuttakeIO() {});
        }
        if (elevator == null) {
          elevator = new Elevator(new ElevatorIO() {});
        }
        if (intake == null) {
          intake = new Intake(new IntakeIO() {});
        }
        if (outtakeRoll == null) {
          outtakeRoll = new OuttakeRoll(new OuttakeRollIO() {});
        }
        if (indexerRoll == null) {
          indexerRoll = new IndexerRoll(new IndexerRollIO() {});
        }
        if (intakeRoll == null) {
          intakeRoll = new IntakeRoll(new IntakeRollIO() {});
        }
        if (rollersSensorsIO == null) {
          rollersSensorsIO = new RollersSensorsIO() {};
        }

        break;
    }

    superstructure = new Superstructure(elevator, outtake, intake);

    rollers = new Rollers(outtakeRoll, indexerRoll, intakeRoll, rollersSensorsIO);
    // Configure the button bindings
    configureButtonBindings();

    configureAutoChooser();
  }

  private void configureAutoChooser() {
    // Add Choreo-based autos
    AutoRoutineBuilder choreoAutos =
        new AutoRoutineBuilder(
            drive.getAutoFactory(), drive, limelightFront, rollers, superstructure);

    // Set up auto routines
    autoChooser.addOption("SCORE L3", choreoAutos.pickupAndScoreAuto().cmd());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> drive.isBrakingEnabled() ? 0.0 : -driverController.getLeftY(),
            () -> drive.isBrakingEnabled() ? 0.0 : -driverController.getLeftX(),
            () -> drive.isBrakingEnabled() ? 0.0 : -driverController.getRightX()));

    // Lock to 0° when A button is held
    driverController
        .x()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> driverController.getLeftY(),
                () -> driverController.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when A button is pressed and disable braking when released
    driverController
        .a()
        .whileTrue(Commands.runOnce(drive::stopWithX, drive))
        .onFalse(Commands.runOnce(drive::disableBrakingMode, drive));

    // Reset gyro to 0° when B button is pressed
    driverController
        .b()
        .onTrue(
            Commands.runOnce(() -> drive.setRotation(new Rotation2d(0.0)), drive)
                .ignoringDisable(true));

    driverController
        .rightBumper()
        .onTrue(Commands.runOnce(() -> drive.increaseSpeedIndex(), drive).ignoringDisable(true));

    driverController
        .leftBumper()
        .whileTrue(
            rollers
                .setGoalCommand(Rollers.Goal.INTAKE)
                .alongWith(
                    superstructure
                        .setGoalCommand(Superstructure.Goal.GROUND_INTAKE)
                        .withName("Ground Intake")));

    driverController
        .rightTrigger()
        .onTrue(
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
                }));

    driverController
        .leftTrigger()
        .onTrue(
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

                  } else {
                    System.out.println(
                        "[MoveToTagPose] No valid target pose found. Skipping movement.");
                  }
                }));

    Trigger l1Trigger = operatorController.a();
    // Score L1
    l1Trigger
        .and(DriverStation::isEnabled)
        .whileTrue(
            rollers
                .setGoalCommand(Rollers.Goal.UP)
                .alongWith(
                    superstructure.setGoalCommand(Superstructure.Goal.L1).withName("Score L1")));

    Trigger l2Trigger = operatorController.x();
    // Score L2
    l2Trigger
        .and(DriverStation::isEnabled)
        .whileTrue(
            rollers
                .setGoalCommand(Rollers.Goal.UP)
                .alongWith(
                    superstructure.setGoalCommand(Superstructure.Goal.L2).withName("Score L2")));

    Trigger l3Trigger = operatorController.b();
    // Score L3
    l3Trigger
        .and(DriverStation::isEnabled)
        .whileTrue(
            rollers
                .setGoalCommand(Rollers.Goal.UP)
                .withTimeout(1)
                .alongWith(
                    superstructure.setGoalCommand(Superstructure.Goal.L3).withName("Score L3")));

    Trigger algaeDownRemove = operatorController.leftBumper();
    Trigger algaeUpRemove = operatorController.rightBumper();
    // Algae Remove
    algaeDownRemove
        .and(DriverStation::isEnabled)
        .whileTrue(
            rollers
                .setGoalCommand(Rollers.Goal.ALGAE)
                .alongWith(
                    superstructure
                        .setGoalCommand(Superstructure.Goal.DOWN_ALGAE)
                        .withName("Down Algae")));
    algaeUpRemove
        .and(DriverStation::isEnabled)
        .whileTrue(
            rollers
                .setGoalCommand(Rollers.Goal.ALGAE)
                .alongWith(
                    superstructure
                        .setGoalCommand(Superstructure.Goal.UP_ALGAE)
                        .withName("Up Algae")));

    Trigger removeCoral = operatorController.rightStick();
    // Remove Coral
    removeCoral
        .and(DriverStation::isEnabled)
        .whileTrue(
            rollers
                .setGoalCommand(Rollers.Goal.IDLE)
                .alongWith(
                    superstructure
                        .setGoalCommand(Superstructure.Goal.REMOVE_CORAL)
                        .withName("Ground Intake")));
    Trigger intakeRoller = operatorController.leftTrigger();
    // Intake Roller
    intakeRoller
        .and(DriverStation::isEnabled)
        .whileTrue(
            new ParallelCommandGroup(
                rollers.setGoalCommand(Rollers.Goal.INTAKE).withName("Roller Intake"),
                superstructure.holdPositionCommand().withName("HoldPosition")));

    Trigger outtakeRoller = operatorController.rightTrigger();
    // Intake Roller
    outtakeRoller
        .and(DriverStation::isEnabled)
        .whileTrue(
            new ParallelCommandGroup(
                rollers.setGoalCommand(Rollers.Goal.OUTTAKE).withName("Outtake Roller"),
                superstructure.holdPositionCommand().withName("HoldPosition")));

    Trigger downElevator = operatorController.y();
    // Intake Roller
    downElevator
        .and(DriverStation::isEnabled)
        .whileTrue(
            new ParallelCommandGroup(
                superstructure
                    .setGoalCommand(Superstructure.Goal.DOWN_ELEVATOR)
                    .withName("Ground Intake")));
  }

  public void checkControllers() {
    operatorDisconnected.set(
        !DriverStation.isJoystickConnected(operatorController.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(operatorController.getHID().getPort()));
    driverDisconnected.set(
        !DriverStation.isJoystickConnected(driverController.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(driverController.getHID().getPort()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
