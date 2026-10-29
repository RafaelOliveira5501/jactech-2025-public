package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.DoubleSupplier;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.junction.Logger;

@RequiredArgsConstructor
public abstract class GenericRollerSystem<G extends GenericRollerSystem.VoltageGoal> {
  public interface VoltageGoal {
    DoubleSupplier getVoltageSupplier();
  }

  public abstract G getGoal();

  private final String name;
  private final GenericRollerSystemIO io;
  protected final GenericRollerSystemIOInputsAutoLogged inputs =
      new GenericRollerSystemIOInputsAutoLogged();
  private final Alert disconnected;
  protected final Timer stateTimer = new Timer();
  private G lastGoal;

  public GenericRollerSystem(String name, GenericRollerSystemIO io) {
    this.name = name;
    this.io = io;

    disconnected = new Alert(name + " motor disconnected!", Alert.AlertType.kWarning);
    stateTimer.start();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(name, inputs);
    disconnected.set(!inputs.connected);

    if (getGoal() != lastGoal) {
      stateTimer.reset();
      lastGoal = getGoal();
    }

    io.runVolts(getGoal().getVoltageSupplier().getAsDouble());
    Logger.recordOutput("Rollers/" + name + "Goal", getGoal().toString());
  }
}
