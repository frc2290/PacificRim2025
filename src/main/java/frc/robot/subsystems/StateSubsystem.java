// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Positions.IntakePosition;
import frc.robot.commands.Positions.L1Position;
import frc.robot.commands.Positions.L2Position;
import frc.robot.commands.Positions.L3Position;
import frc.robot.commands.Positions.L4Position;
import frc.robot.commands.Positions.TravelPosition;
import frc.utils.FLYTLib.FLYTDashboard.FlytLogger;

public class StateSubsystem extends SubsystemBase {

  private ElevatorSubsystem elevator;
  private DifferentialSubsystem diff;
  private DriveSubsystem drive;

  public enum State {
    TravelPosition,
    IntakePosition,
    L1Position,
    L2Position,
    L3Position,
    L4Position,
    Transition,
    StartPosition
  }

  private State prevState = State.StartPosition;
  private State currentState = State.StartPosition;
  private State goalState = State.TravelPosition;

  private boolean transitioning = false;

  private DriverStation driverStation;

  private FlytLogger stateDash = new FlytLogger("State");

  /** Creates a new StateSubsystem. */
  public StateSubsystem(DifferentialSubsystem m_diff, ElevatorSubsystem m_elevator, DriveSubsystem m_drive) {
    diff = m_diff;
    elevator = m_elevator;
    drive = m_drive;

    stateDash.addStringPublisher("Current State", false, () -> getCurrentState().toString());
    stateDash.addStringPublisher("Prev State", false, () -> getPrevState().toString());
    stateDash.addStringPublisher("Goal State", false, () -> getGoalState().toString());
    stateDash.addBoolPublisher("Transitioning", false, () -> isTransitioning());
  }

  public boolean isTransitioning() {
    return transitioning;
  }

  public State getCurrentState() {
    return currentState;
  }

  public State getPrevState() {
    return prevState;
  }

  public State getGoalState() {
    return goalState;
  }

  public void setCurrentState(State curState) {
    transitioning = false;
    prevState = currentState;
    currentState = curState;
    System.out.println("Current: " + currentState.toString() + " Prev: " + prevState.toString() + " Goal: " + goalState.toString());
  }

  public void setGoal(State newState) {
    goalState = newState;
    System.out.println("New Goal: " + newState.toString());
    //currentState = newState;
  }

  public Command setGoalCommand(State newGoal) {
    return this.runOnce(() -> goalState = newGoal);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (goalState != currentState && !isTransitioning() && driverStation.isEnabled()) {
      switch(goalState) {
        case TravelPosition:
          new TravelPosition(diff, elevator, this).schedule();
          break;
        case IntakePosition:
          if (currentState != State.TravelPosition) {
            new TravelPosition(diff, elevator, this).schedule();
          } else {
            new IntakePosition(diff, elevator, this).schedule();
          }
          break;
        case L1Position:
          if (currentState == State.IntakePosition) {
            new TravelPosition(diff, elevator, this).schedule();
          } else {
            new L1Position(diff, elevator, drive, this).schedule();
          }
          break;
        case L2Position:
          if (currentState == State.IntakePosition) {
            new TravelPosition(diff, elevator, this).schedule();
          } else {
            new L2Position(diff, elevator, drive, this).schedule();
          }
          break;
        case L3Position:
          if (currentState == State.IntakePosition) {
            new TravelPosition(diff, elevator, this).schedule();
          } else {
            new L3Position(diff, elevator, drive, this).schedule();
          }
          break;
        case L4Position:
          if (currentState == State.IntakePosition) {
            new TravelPosition(diff, elevator, this).schedule();
          } else {
            new L4Position(diff, elevator, drive, this).schedule();
          }
          break;
        case Transition:
          System.out.println("Transitioning!!!!!!!!!!!!!");
          break;
        case StartPosition:
          new TravelPosition(diff, elevator, this).schedule();
          break;
        default:
          System.out.println("Unknown State!!!!!!!!!!");
          break;
      }
      transitioning = true;
    }
    stateDash.update(Constants.debugMode);
  }
}
