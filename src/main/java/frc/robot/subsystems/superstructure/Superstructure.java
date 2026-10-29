// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.Elevator.Elevator;
import frc.robot.subsystems.superstructure.Intake.Intake;
// import frc.robot.subsystems.superstructure.Intake.Intake;
import frc.robot.subsystems.superstructure.Outtake.Outtake;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class Superstructure extends SubsystemBase {
  @Getter
  public enum Goal {
    STOW,
    GROUND_INTAKE,
    DOWN_ELEVATOR,
    REMOVE_CORAL,
    UP_ALGAE,
    DOWN_ALGAE,
    L1,
    L2,
    L3,
    L4,
    RESET
  }

  @Getter private Goal currentGoal = Goal.STOW;
  @Getter private Goal desiredGoal = Goal.STOW;
  private Goal lastGoal = Goal.STOW;

  private Outtake outtake;
  private Elevator elevator;
  private Intake intake;
  private Timer goalTimer = new Timer();

  public Superstructure(Elevator elevator, Outtake outtake, Intake intake) {
    this.elevator = elevator;
    this.outtake = outtake;
    this.intake = intake;

    setDefaultCommand(setGoalCommand(Goal.STOW));
    goalTimer.start();
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      setDefaultCommand(setGoalCommand(Goal.STOW));
    }

    currentGoal = desiredGoal;

    if (currentGoal != lastGoal) {
      goalTimer.reset();
    }
    lastGoal = currentGoal;

    switch (currentGoal) {
      case STOW -> {
        elevator.setGoal(Elevator.Goal.STOW);
        outtake.setGoal(Outtake.Goal.STOW);
        intake.setGoal(Intake.Goal.STOW);
      }
      case GROUND_INTAKE -> {
        elevator.setGoal(Elevator.Goal.GROUND_INTAKE);
        outtake.setGoal(Outtake.Goal.GROUND_INTAKE);
        intake.setGoal(Intake.Goal.GROUND_INTAKE);
      }
      case DOWN_ELEVATOR -> {
        elevator.setGoal(Elevator.Goal.DOWN_ELEVATOR);
        outtake.setGoal(Outtake.Goal.DOWN_ELEVATOR);
      }
      case L1 -> {
        elevator.setGoal(Elevator.Goal.SCORE_L1);
        outtake.setGoal(Outtake.Goal.SCORE_L1);
      }
      case L2 -> {
        elevator.setGoal(Elevator.Goal.SCORE_L2);
        outtake.setGoal(Outtake.Goal.SCORE_L2);
      }
      case L3 -> {
        elevator.setGoal(Elevator.Goal.SCORE_L3);
        outtake.setGoal(Outtake.Goal.SCORE_L3);
      }
      case UP_ALGAE -> {
        elevator.setGoal(Elevator.Goal.UP_ALGAE);
        outtake.setGoal(Outtake.Goal.UP_ALGAE);
      }
      case DOWN_ALGAE -> {
        elevator.setGoal(Elevator.Goal.LOW_ALGAE);
        outtake.setGoal(Outtake.Goal.LOW_ALGAE);
      }
      case REMOVE_CORAL -> {
        elevator.setGoal(Elevator.Goal.REMOVE_CORAL);
        outtake.setGoal(Outtake.Goal.STOW);
      }
      case RESET -> {
        desiredGoal = Goal.STOW;
        setDefaultCommand(setGoalCommand(Goal.STOW));
      }
    }

    elevator.periodic();
    outtake.periodic();
    intake.periodic();

    Logger.recordOutput("Superstructure/GoalState", desiredGoal);
    Logger.recordOutput("Superstructure/CurrentState", currentGoal);
  }

  /** Set goal of superstructure */
  private void setGoal(Goal goal) {
    if (desiredGoal == goal) return;
    desiredGoal = goal;
  }

  /** Command to set goal of superstructure */
  public Command setGoalCommand(Goal goal) {
    return startEnd(() -> setGoal(goal), () -> setGoal(Goal.STOW))
        .withName("Supersctructure" + goal);
  }

  public Command holdPositionCommand() {
    return Commands.run(() -> setGoal(getCurrentGoal()), this);
  }

  @AutoLogOutput(key = "Superstructure/CompletedGoal")
  public boolean atGoal() {
    return currentGoal == desiredGoal
        //  && intake.atGoal()
        && outtake.atGoal();
  }
}
