package frc.robot.subsystems.superstructure.Elevator;

import static frc.robot.subsystems.superstructure.Elevator.ElevatorConstants.*;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import java.util.List;

/** Implementation of the ElevatorIO interface for real hardware using Kraken motors. */
public class ElevatorIONeo implements ElevatorIO {
  // Status Signals
  private double internalPositionRotations;
  private double velocityRps;
  private double supplyCurrentAmps;
  private List<Double> appliedVoltage;
  private List<Double> tempCelsius;

  private final SparkMax leaderElevatorMotor;
  private final SparkMax followerElevatorMotor;

  SparkMaxConfig configLeader = new SparkMaxConfig();
  SparkMaxConfig configFollower = new SparkMaxConfig();

  public ElevatorIONeo() {

    leaderElevatorMotor = new SparkMax(leaderElevatorCanId, MotorType.kBrushless);
    followerElevatorMotor = new SparkMax(followerElevatorCanId, MotorType.kBrushless);

    // Configure follower
    configFollower.follow(leaderElevatorMotor, true);
    configFollower.idleMode(IdleMode.kBrake);

    // Configure leader
    configLeader
        .encoder
        .positionConversionFactor(0.05 * Math.PI / 25)
        .velocityConversionFactor(0.05 * Math.PI / 25);
    configLeader
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(gains.kP(), gains.kI(), gains.kD())
        .maxMotion
        .maxVelocity(profileConstraints.maxVelocity)
        .maxAcceleration(profileConstraints.maxAcceleration)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

    configLeader.idleMode(IdleMode.kBrake);
    configLeader.inverted(true);

    leaderElevatorMotor.configure(
        configLeader, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followerElevatorMotor.configure(
        configFollower, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

    // Status signals setup
    internalPositionRotations = leaderElevatorMotor.getEncoder().getPosition();
    velocityRps = leaderElevatorMotor.getEncoder().getVelocity();
    appliedVoltage =
        List.of(leaderElevatorMotor.getAppliedOutput(), followerElevatorMotor.getAppliedOutput());
    tempCelsius =
        List.of(
            leaderElevatorMotor.getMotorTemperature(), followerElevatorMotor.getMotorTemperature());
    supplyCurrentAmps = leaderElevatorMotor.getOutputCurrent();
    // Refresh all signals and update inputs
    inputs.leaderMotorConnected = !leaderElevatorMotor.hasActiveFault();
    inputs.followerMotorConnected = !followerElevatorMotor.hasActiveFault();

    // Convert rotations to meters for position and velocity
    inputs.positionMeters = internalPositionRotations;
    inputs.velocityMetersPerSec = velocityRps;
    inputs.supplyCurrentAmps = supplyCurrentAmps;
    // Map voltages, currents, and temperatures
    inputs.appliedVolts = appliedVoltage.get(0);
    inputs.tempCelcius = tempCelsius.get(0);
  }

  @Override
  public void stop() {
    // Stop the elevator
    leaderElevatorMotor.set(0);
  }

  @Override
  public void setPID(double p, double i, double d) {
    // Update PID gains dynamically
    configLeader
        .closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(gains.kP(), gains.kI(), gains.kD());
  }

  @Override
  public void runSetpoint(double setpoint, double feedforward) {
    // Convert setpoint from meters to rotations and run position control
    leaderElevatorMotor
        .getClosedLoopController()
        .setReference(
            setpoint, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, feedforward);
  }
}
