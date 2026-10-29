package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.constants.DriveControlLoops;
import frc.robot.subsystems.vision.VisionIOSim.TimestampedPose;
import frc.robot.util.ChassisHeadingController;
import frc.robot.util.LocalADStarAK;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  private boolean brakingEnabled = false;
  private final List<TimestampedPose> poseHistory = new ArrayList<>();
  public final AutoFactory autoFactory;

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;

  private final double[] speedMultipliers = {0.10, 0.25, 0.5, 0.75, 1.0}; // Speed levels
  private int currentSpeedIndex = speedMultipliers.length - 1; // Default to max speed

  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    SparkOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(2.0, 0.0, 0.0), // Translation: Lower from 5.0 to 2.0
            new PIDConstants(1.5, 0.0, 0.0)), // Rotation: Lower from 5.0 to 1.5
        ppConfig,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);

    autoFactory =
        new AutoFactory(
            this::getPose,
            this::setPose,
            this::followTrajectory,
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            this);

    headingController.enableContinuousInput(-Math.PI, Math.PI);

    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  public AutoFactory getAutoFactory() {
    return autoFactory;
  }

  @Override
  public void periodic() {
    // Add current pose to history
    poseHistory.add(new TimestampedPose(Timer.getFPGATimestamp(), getPose()));

    // Remove entries older than 5 seconds
    double cutoff = Timer.getFPGATimestamp() - 5.0;
    poseHistory.removeIf(entry -> entry.timestamp < cutoff);

    odometryLock.lock();
    gyroIO.updateInputs(gyroInputs);
    Logger.recordOutput("Drive/rawGyroRotation", rawGyroRotation);
    Logger.processInputs("Drive/Gyro", gyroInputs);

    for (var module : modules) {
      module.periodic();
    }

    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Get the current speed multiplier
    double multiplier = getCurrentSpeedMultiplier();

    // Scale the speeds by the multiplier
    ChassisSpeeds scaledSpeeds =
        new ChassisSpeeds(
            speeds.vxMetersPerSecond * multiplier,
            speeds.vyMetersPerSecond * multiplier,
            speeds.omegaRadiansPerSecond * multiplier);

    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(scaledSpeeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxSpeedMetersPerSec);

    // Log unoptimized setpoints
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    brakingEnabled = true; // Prevent joystick control
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = moduleTranslations[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
    System.out.println("Stopped with X configuration");
  }

  /** Disables braking mode, allowing joystick control again. */
  public void disableBrakingMode() {
    brakingEnabled = false;
    System.out.println("Braking mode disabled");
  }

  /** Check if braking mode is active. */
  public boolean isBrakingEnabled() {
    return brakingEnabled;
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rad/sec. */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Resets the current odometry rotation while preserving x and y. */
  public void setRotation(Rotation2d rotation) {
    Pose2d currentPose = poseEstimator.getEstimatedPosition();
    Pose2d newPose = new Pose2d(currentPose.getTranslation(), rotation);
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), newPose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  public void mergeVisionPose(Pose2d visionPose) {
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      visionPose = visionPose.relativeTo(new Pose2d(17.55, 8.05, new Rotation2d()));
    }
    // Trust vision X/Y but not rotation
    Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0.1, 0.1, 10.0);
    addVisionMeasurement(visionPose, Timer.getFPGATimestamp(), visionStdDevs);
    Logger.recordOutput("Odometry/Vision Pose + Swerve Rotation", visionPose);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return maxSpeedMetersPerSec;
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return maxSpeedMetersPerSec / driveBaseRadius;
  }

  /** Increases swerve speed */
  public void increaseSpeedIndex() {
    if (currentSpeedIndex < speedMultipliers.length - 1) {
      currentSpeedIndex++;
    }
    System.out.println("Speed increased: " + speedMultipliers[currentSpeedIndex]);
  }

  /** Decreases swerve speed */
  public void decreaseSpeedIndex() {
    if (currentSpeedIndex > 0) {
      currentSpeedIndex--;
    }
    System.out.println("Speed decreased: " + speedMultipliers[currentSpeedIndex]);
  }

  public double getCurrentSpeedMultiplier() {
    return speedMultipliers[currentSpeedIndex];
  }

  public TimestampedPose getHistoricalPose(double targetTimestamp) {
    if (poseHistory.isEmpty()) return new TimestampedPose(Timer.getFPGATimestamp(), getPose());

    // Linear search for closest timestamp (replace with binary search for better performance)
    TimestampedPose bestMatch = poseHistory.get(0);
    for (TimestampedPose entry : poseHistory) {
      if (Math.abs(entry.timestamp - targetTimestamp)
          < Math.abs(bestMatch.timestamp - targetTimestamp)) {
        bestMatch = entry;
      }
    }
    return bestMatch;
  }

  public Rotation2d getFacing() {
    return getPose().getRotation();
  }

  public double getChassisMaxAngularVelocity() {
    return maxSpeedMetersPerSec / 0.45;
  }
  ;

  public double getChassisMaxLinearVelocityMetersPerSec() {
    return maxSpeedMetersPerSec;
  }
  ;

  public double getChassisMaxAngularAccelerationRadPerSecSq() {
    return 3.56; // Rad/s²;
  }
  ;

  public double getChassisMaxAccelerationMetersPerSecSq() {
    return 1.6; // m/s;
  }
  ;

  public ChassisSpeeds getMeasuredChassisSpeedsFieldRelative() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getFacing());
  }

  public static final ChassisHeadingController swerveHeadingController =
      new ChassisHeadingController(
          new TrapezoidProfile.Constraints(maxSpeedMetersPerSec, 3.56),
          DriveControlLoops.CHASSIS_ROTATION_CLOSE_LOOP,
          new Rotation2d());

  public void runRobotCentricChassisSpeeds(ChassisSpeeds speeds) {
    final double PERCENT_DEADBAND = 0.03;
    if (Math.abs(speeds.omegaRadiansPerSecond) < PERCENT_DEADBAND * getChassisMaxAngularVelocity()
        && Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
            < PERCENT_DEADBAND * getChassisMaxLinearVelocityMetersPerSec())
      speeds = new ChassisSpeeds();

    runVelocity(ChassisSpeeds.discretize(speeds, 0.02));
  }

  public PathConstraints getChassisConstrains(double speedMultiplier) {
    return new PathConstraints(
        getChassisMaxLinearVelocityMetersPerSec() * speedMultiplier,
        getChassisMaxAccelerationMetersPerSecSq(),
        getChassisMaxAngularVelocity() * speedMultiplier,
        getChassisMaxAngularAccelerationRadPerSecSq());
  }

  private final PIDController xController = new PIDController(10.0, 0.0, 0.0);
  private final PIDController yController = new PIDController(10.0, 0.0, 0.0);
  private final PIDController headingController = new PIDController(7.5, 0.0, 0.0);

  public void followTrajectory(SwerveSample sample) {
    // Get the current pose of the robot
    Pose2d pose = getPose();

    // Generate the next speeds for the robot
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega
                + headingController.calculate(pose.getRotation().getRadians(), sample.heading));

    // Apply the generated speeds
    runVelocity(speeds);
  }
}
