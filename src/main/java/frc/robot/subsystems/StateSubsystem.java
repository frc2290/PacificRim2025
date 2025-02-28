// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

  private FlytLogger stateDash = new FlytLogger("State");

  /** Creates a new StateSubsystem. */
  public StateSubsystem(DifferentialSubsystem m_diff, ElevatorSubsystem m_elevator) {
    diff = m_diff;
    elevator = m_elevator;

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
    prevState = currentState;
    currentState = curState;
    transitioning = false;
  }

  public void setGoal(State newState) {
    goalState = newState;
    //currentState = newState;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (goalState != currentState && !isTransitioning()) {
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
          if (prevState == State.IntakePosition) {
            new TravelPosition(diff, elevator, this).schedule();
          } else {
            new L1Position(diff, elevator, this).schedule();
          }
          break;
        case L2Position:
          if (prevState == State.IntakePosition) {
            new TravelPosition(diff, elevator, this).schedule();
          } else {
            new L2Position(diff, elevator, this).schedule();
          }
          break;
        case L3Position:
          if (prevState == State.IntakePosition) {
            new TravelPosition(diff, elevator, this).schedule();
          } else {
            new L3Position(diff, elevator, this).schedule();
          }
          break;
        case L4Position:
          if (prevState == State.IntakePosition) {
            new TravelPosition(diff, elevator, this).schedule();
          } else {
            new L4Position(diff, elevator, this).schedule();
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
