package frc.robot.subsystems.rollers;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;

/** Generic roller IO implementation for a roller or series of rollers using a NEO. */
public class IndexerRollerSystemIONEO implements GenericRollerSystemIO {
  private final SparkMax sparkMaxLeader;
  private final SparkMax sparkMaxFollower;
  private final SparkMaxConfig configLeader = new SparkMaxConfig();
  private final SparkMaxConfig configFollower = new SparkMaxConfig();

  private final double reduction;

  public IndexerRollerSystemIONEO(
      int idLeader,
      int idFollower,
      int currentLimitAmps,
      boolean invert,
      boolean brake,
      double reduction) {
    this.reduction = reduction;

    sparkMaxLeader = new SparkMax(idLeader, MotorType.kBrushless);
    sparkMaxFollower = new SparkMax(idFollower, MotorType.kBrushless);
    configFollower.follow(idLeader, invert);
    configLeader.idleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    sparkMaxLeader.configure(
        configLeader, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    sparkMaxFollower.configure(
        configFollower, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(GenericRollerSystemIOInputs inputs) {
    inputs.connected = sparkMaxLeader.getFirmwareVersion() != 0; // Verifica conexão
    inputs.velocityRadsPerSec =
        Units.rotationsToRadians(sparkMaxLeader.get() / 60.0) / reduction; // RPM -> rad/s
    inputs.appliedVoltage = sparkMaxLeader.getBusVoltage() * sparkMaxLeader.getAppliedOutput();
    inputs.supplyCurrentAmps = sparkMaxLeader.getOutputCurrent();
    inputs.tempCelsius = sparkMaxLeader.getMotorTemperature();
  }

  @Override
  public void runVolts(double volts) {
    sparkMaxLeader.setVoltage(volts);
  }

  @Override
  public void stop() {
    sparkMaxLeader.stopMotor();
  }
}
