package frc.robot.subsystems.rollers;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;

/** Generic roller IO implementation for a roller or series of rollers using a SPARK Flex. */
public abstract class GenericRollerSystemIOSparkFlex implements GenericRollerSystemIO {
  private final SparkMax motor;

  private final double reduction;

  public GenericRollerSystemIOSparkFlex(
      int id, int currentLimitAmps, boolean invert, boolean brake, double reduction) {
    this.reduction = reduction;
    motor = new SparkMax(id, SparkBase.MotorType.kBrushless);

    // motor.setSmartCurrentLimit(currentLimitAmps);
    // motor.setInverted(invert);
    // motor.setIdleMode(brake ? SparkBase.ResetMode.kNoResetSafeParameters :
    // SparkBase.ResetMode.kResetSafeParameters);

  }

  public void updateInputs(GenericRollerSystemIOInputs inputs) {
    inputs.appliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.supplyCurrentAmps = motor.getOutputCurrent();
    inputs.tempCelsius = motor.getMotorTemperature();
  }

  @Override
  public void runVolts(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
