package frc.robot.subsystems.rollers;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;

/** Generic roller IO implementation for a roller or series of rollers using a NEO. */
public class GenericRollerSystemIONEO implements GenericRollerSystemIO {
  private final SparkMax sparkMax;
  private final SparkMaxConfig config = new SparkMaxConfig();

  private final double reduction;

  public GenericRollerSystemIONEO(
      int id, String bus, int currentLimitAmps, boolean invert, boolean brake, double reduction) {
    this.reduction = reduction;

    sparkMax = new SparkMax(id, MotorType.kBrushless);
    config.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    sparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(GenericRollerSystemIOInputs inputs) {
    inputs.connected = sparkMax.getFirmwareVersion() != 0; // Verifica conexão
    inputs.velocityRadsPerSec =
        Units.rotationsToRadians(sparkMax.get() / 60.0) / reduction; // RPM -> rad/s
    inputs.appliedVoltage = sparkMax.getBusVoltage() * sparkMax.getAppliedOutput();
    inputs.supplyCurrentAmps = sparkMax.getOutputCurrent();
    inputs.tempCelsius = sparkMax.getMotorTemperature();
  }

  @Override
  public void runVolts(double volts) {
    sparkMax.setVoltage(volts);
  }

  @Override
  public void stop() {
    sparkMax.stopMotor();
  }
}
