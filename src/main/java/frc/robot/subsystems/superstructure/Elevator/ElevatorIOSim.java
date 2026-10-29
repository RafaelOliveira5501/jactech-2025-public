package frc.robot.subsystems.superstructure.Elevator;

import static frc.robot.subsystems.superstructure.Elevator.ElevatorConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

/** Simulation implementation of the ElevatorIO interface. */
public class ElevatorIOSim implements ElevatorIO {
  private final double drumRadiusMeters = 0.02; // Raio do tambor em metros (exemplo: 2 cm)
  private final double carriageMassKg = 3.0; // Massa do carro do elevador em kg

  private final ElevatorSim sim =
      new ElevatorSim(
          DCMotor.getKrakenX60(2), // Motor Kraken com 2 motores
          reduction, // Redução do sistema
          carriageMassKg, // Massa do carro
          drumRadiusMeters, // Raio do tambor
          0.0, // Altura mínima
          maxLengthMeters, // Altura máxima
          true, // Adiciona ruído à simulação
          0.0 // Voltagem inicial (começa desligado)
          );

  private final PIDController controller;
  private double appliedVoltage = 0.0; // Voltagem aplicada ao motor
  private double positionOffset = 0.0; // Offset inicial da posição (para resets)

  private boolean controllerNeedsReset = false; // Indica se o controlador precisa de reset
  private boolean closedLoop = true; // Indica se o controle está em malha fechada
  private boolean wasNotAuto = true; // Indica se saiu de um estado automático

  public ElevatorIOSim() {
    // Configura o controlador PID com valores padrão
    controller = new PIDController(0.0, 0.0, 0.0);
    sim.setState(0.0, 0.0); // Inicializa a posição inicial do simulador
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // Reset do controlador se o robô estiver desabilitado
    if (DriverStation.isDisabled()) {
      controllerNeedsReset = true;
    }

    // Configuração do simulador para início de modo autônomo
    if (wasNotAuto && DriverStation.isAutonomousEnabled()) {
      sim.setState(0.0, 0.0); // Reseta a posição no modo autônomo
      wasNotAuto = false;
    }
    wasNotAuto = !DriverStation.isAutonomousEnabled();

    // Atualiza o estado do simulador
    sim.update(Constants.loopPeriodSecs);

    // Preenche os inputs com os valores do simulador
    inputs.positionMeters = sim.getPositionMeters() + positionOffset;
    inputs.velocityMetersPerSec = sim.getVelocityMetersPerSecond();
    inputs.appliedVolts = appliedVoltage;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.torqueCurrentAmps = sim.getCurrentDrawAmps(); // Simula torque igual ao corrente
    inputs.tempCelcius = 0.0; // Temperatura não é simulada
  }

  @Override
  public void runSetpoint(double setpoint, double feedforward) {
    // Muda para controle em malha fechada
    if (!closedLoop) {
      controllerNeedsReset = true;
      closedLoop = true;
    }
    if (controllerNeedsReset) {
      controller.reset();
      controllerNeedsReset = false;
    }

    // Calcula a voltagem necessária para atingir o setpoint
    double voltage =
        controller.calculate(sim.getPositionMeters(), setpoint + positionOffset) + feedforward;
    runVolts(voltage);
  }

  @Override
  public void runVolts(double volts) {
    // Define a voltagem aplicada ao simulador (malha aberta)
    closedLoop = false;
    appliedVoltage = Math.min(Math.max(volts, -12), 12); // Clampa entre -12V e 12V
    sim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void setPID(double p, double i, double d) {
    // Atualiza os valores PID dinamicamente
    controller.setPID(p, i, d);
  }

  @Override
  public void stop() {
    // Para o simulador (zera a voltagem aplicada)
    appliedVoltage = 0.0;
    sim.setInputVoltage(appliedVoltage);
  }

  /**
   * Define a posição inicial do simulador, ajustando o offset.
   *
   * @param position Nova posição inicial em metros
   */
  private void setPosition(double position) {
    positionOffset = position - sim.getPositionMeters();
  }
}
