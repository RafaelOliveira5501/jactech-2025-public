// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.Intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.superstructure.Intake.IntakeConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class IntakeIOKraken implements IntakeIO {

  //  Hardware
  private final TalonFX leaderMotor;
  private final TalonFX followerMotor;
  private final CANcoder absoluteEncoder;

  //  Status Signal
  private final StatusSignal<Angle> internalPositionRotations;
  private final StatusSignal<Angle> encoderAbsolutePositionRotations;
  private final StatusSignal<AngularVelocity> velocityRps;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> tempCelsius;

  // Control
  private final MotionMagicVoltage positionControl =
      new MotionMagicVoltage(0).withUpdateFreqHz(0.0);

  // Config
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  public IntakeIOKraken() {
    leaderMotor = new TalonFX(leaderCanId, "rio");
    followerMotor = new TalonFX(followerCanId, "rio");
    absoluteEncoder = new CANcoder(intakeEncoderID, "rio");

    followerMotor.setControl(new Follower(leaderMotor.getDeviceID(), true));

    // IntakeMotor Configs
    config.Slot0.kP = gains.kP();
    config.Slot0.kI = gains.kI();
    config.Slot0.kD = gains.kD();
    config.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    /* Configura o CANCoder como sensor de feedback */
    FeedbackConfigs fdb = config.Feedback;
    fdb.FeedbackRemoteSensorID = absoluteEncoder.getDeviceID(); // ID do CANCoder
    fdb.FeedbackSensorSource =
        FeedbackSensorSourceValue.RemoteCANcoder; // Usa CANCoder como feedback
    fdb.SensorToMechanismRatio = 1;

    /* Configura o Motion Magic */
    MotionMagicConfigs mm = config.MotionMagic;
    mm.withMotionMagicCruiseVelocity(profileConstraints.maxVelocity) // 5 rotações por segundo
        .withMotionMagicAcceleration(
            RotationsPerSecondPerSecond.of(
                profileConstraints
                    .maxAcceleration)) // Aceleração de 10 rotações por segundo ao quadrado
        .withMotionMagicJerk(
            RotationsPerSecondPerSecond.per(Second)
                .of(100)); // Arrancada de 100 rotações por segundo ao cubo

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = leaderMotor.getConfigurator().apply(config);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Erro ao configurar o TalonFX: " + status.toString());
    }

    absoluteEncoder.getAbsolutePosition().refresh(); // Force a fresh read
    Angle absolutePosition =
        absoluteEncoder.getAbsolutePosition().getValue(); // Sincroniza com a posição absoluta
    absoluteEncoder.setPosition(absolutePosition);

    // Status Signal
    internalPositionRotations = leaderMotor.getPosition();
    encoderAbsolutePositionRotations = absoluteEncoder.getAbsolutePosition();
    velocityRps = leaderMotor.getVelocity();
    appliedVoltage = leaderMotor.getMotorVoltage();
    supplyCurrent = leaderMotor.getSupplyCurrent();
    torqueCurrent = leaderMotor.getTorqueCurrent();
    tempCelsius = leaderMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        internalPositionRotations,
        velocityRps,
        appliedVoltage,
        supplyCurrent,
        torqueCurrent,
        tempCelsius);

    BaseStatusSignal.setUpdateFrequencyForAll(50, encoderAbsolutePositionRotations);
  }

  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeMotorConnected =
        BaseStatusSignal.refreshAll(
                internalPositionRotations,
                velocityRps,
                appliedVoltage,
                supplyCurrent,
                torqueCurrent,
                tempCelsius)
            .isOK();
    inputs.positionRads = Units.rotationsToRadians(internalPositionRotations.getValueAsDouble());
    inputs.absoluteEncoderConnected =
        BaseStatusSignal.refreshAll(encoderAbsolutePositionRotations).isOK();
    double rawRotations = encoderAbsolutePositionRotations.getValueAsDouble();
    double adjustedRotations = rawRotations;
    inputs.absoluteEncoderPositionRads = Units.rotationsToRadians(adjustedRotations);
    inputs.velocityRadsPerSec = Units.rotationsToRadians(velocityRps.getValueAsDouble());
    inputs.appliedVolts = appliedVoltage.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.torqueCurrentAmps = torqueCurrent.getValueAsDouble();
    inputs.tempCelcius = tempCelsius.getValueAsDouble();
  }

  @Override
  public void runSetpoint(double setpointRads, double feedforward) {
    leaderMotor.setControl(
        positionControl
            .withPosition(Units.radiansToRotations(setpointRads))
            .withFeedForward(feedforward)
            .withSlot(0));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    leaderMotor.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setPID(double p, double i, double d) {
    config.Slot0.kP = p;
    config.Slot0.kI = i;
    config.Slot0.kD = d;
    leaderMotor.getConfigurator().apply(config, 0.01);
  }

  @Override
  public void stop() {
    leaderMotor.setControl(new NeutralOut());
  }
}
