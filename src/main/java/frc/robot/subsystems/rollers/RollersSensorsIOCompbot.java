package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

public class RollersSensorsIOCompbot implements RollersSensorsIO {
  private final CANrange outtakeStaged = new CANrange(16);
  private final CANrangeConfiguration outtakeStagedConfigurator = new CANrangeConfiguration();

  public RollersSensorsIOCompbot() {
    outtakeStagedConfigurator.FovParams.FOVRangeX = 3;
    outtakeStagedConfigurator.FovParams.FOVRangeY = 3;
    outtakeStagedConfigurator.ProximityParams.ProximityThreshold = 0.2;
    outtakeStagedConfigurator.ProximityParams.ProximityHysteresis = 0.01;
  }

  @Override
  public void updateInputs(RollersSensorsIOInputs inputs) {
    inputs.sensorDistance = outtakeStaged.getDistance().getValueAsDouble();
    inputs.outtakeStaged = (outtakeStaged.getDistance().getValueAsDouble() < 0.1);
  }
}
