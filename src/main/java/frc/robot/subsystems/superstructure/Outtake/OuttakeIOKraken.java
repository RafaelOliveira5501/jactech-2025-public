// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.Outtake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.superstructure.Outtake.OuttakeConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class OuttakeIOKraken implements OuttakeIO {

  //  Hardware
  private final TalonFX outtakeMotor;
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

  public OuttakeIOKraken() {
    outtakeMotor = new TalonFX(OuttakeConstants.outtakeCanId, "rio");
    absoluteEncoder = new CANcoder(OuttakeConstants.outtakeEncoderID, "rio");

    // OuttakeMotor Configs
    config.Slot0.kP = OuttakeConstants.gains.kP();
    config.Slot0.kI = OuttakeConstants.gains.kI();
    config.Slot0.kD = OuttakeConstants.gains.kD();
    config.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
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
      status = outtakeMotor.getConfigurator().apply(config);
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
    internalPositionRotations = outtakeMotor.getPosition();
    encoderAbsolutePositionRotations = absoluteEncoder.getAbsolutePosition();
    velocityRps = outtakeMotor.getVelocity();
    appliedVoltage = outtakeMotor.getMotorVoltage();
    supplyCurrent = outtakeMotor.getSupplyCurrent();
    torqueCurrent = outtakeMotor.getTorqueCurrent();
    tempCelsius = outtakeMotor.getDeviceTemp();

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

  public void updateInputs(OuttakeIOInputs inputs) {
    inputs.outtakeMotorConnected =
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
    outtakeMotor.setControl(
        positionControl
            .withPosition(Units.radiansToRotations(setpointRads))
            .withFeedForward(feedforward)
            .withSlot(0));
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    outtakeMotor.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setPID(double p, double i, double d) {
    config.Slot0.kP = p;
    config.Slot0.kI = i;
    config.Slot0.kD = d;
    outtakeMotor.getConfigurator().apply(config, 0.01);
  }

  @Override
  public void stop() {
    outtakeMotor.setControl(new NeutralOut());
  }
}
