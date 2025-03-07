// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Manipulator;
import frc.robot.commands.IntakeCoral;
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
    private ManipulatorSubsystem manipulator;

    public enum State {
        TravelPosition,
        IntakePosition,
        L1Position,
        L2Position,
        L3Position,
        L4Position,
        StartPosition,
        Cancelled
    }

    public enum DriveState {
        ReefScore,
        NetScore,
        ProcessorScore,
        CoralStation,
        Climb,
        Teleop
    }

    private boolean rotLock = true;
    private DriveState driveState = DriveState.NetScore;

    private boolean rightScore = false;

    private State prevState = State.StartPosition;
    private State currentState = State.StartPosition;
    private State goalState = State.TravelPosition;

    private boolean transitioning = false;

    private Command currentCommand = null;

    private FlytLogger stateDash = new FlytLogger("State");

    /** Creates a new StateSubsystem. */
    public StateSubsystem(DifferentialSubsystem m_diff, ElevatorSubsystem m_elevator, DriveSubsystem m_drive, ManipulatorSubsystem m_manip) {
        diff = m_diff;
        elevator = m_elevator;
        drive = m_drive;
        manipulator = m_manip;

        stateDash.addStringPublisher("Current State", false, () -> getCurrentState().toString());
        stateDash.addStringPublisher("Prev State", false, () -> getPrevState().toString());
        stateDash.addStringPublisher("Goal State", false, () -> getGoalState().toString());
        stateDash.addBoolPublisher("Transitioning", false, () -> isTransitioning());
        stateDash.addBoolPublisher("At State", false, () -> atCurrentState());
        stateDash.addStringPublisher("Drive State", false, () -> getDriveState().toString());
        stateDash.addBoolPublisher("Rotation Lock", false, () -> getRotationLock());
    }

    /** Robot State Section */

    public boolean isTransitioning() {
        return transitioning;
    }

    public State getCurrentState() {
        return currentState;
    }

    public boolean atCurrentState() {
        return elevator.atPosition() && diff.atExtenstionSetpoint() && diff.atRotationSetpoint();
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
        System.out.println("Current: " + currentState.toString() + " Prev: " + prevState.toString() + " Goal: "
                + goalState.toString());
    }

    public void setGoal(State newState) {
        goalState = newState;
        System.out.println("New Goal: " + newState.toString());
        // currentState = newState;
    }

    public void cancelCurrentCommand() {
        currentCommand.cancel();
        setCurrentState(State.Cancelled);
        setGoal(State.Cancelled);
        elevator.setElevatorSetpoint(elevator.getPosition());
        diff.setExtensionSetpoint(diff.getExtensionPosition());
        diff.setRotationSetpoint(diff.getRotationPosition());
    }

    public Command cancelCommand() {
        return this.runOnce(() -> cancelCurrentCommand());
    }

    public Command setGoalCommand(State newGoal) {
        return this.runOnce(() -> goalState = newGoal);
    }

    public boolean getRightScore() {
        return rightScore;
    }

    public void setRightScore(boolean right) {
        rightScore = right;
    }

    public Command setRightScoreCommand(boolean right) {
        return this.runOnce(() -> rightScore = right);
    }

    public boolean hasCoral() {
        return manipulator.hasCoral();
    }

    /** Drive State Section */
    
    public boolean getRotationLock() {
        return rotLock;
    }

    public void setRotationLock(boolean lock) {
        rotLock = lock;
    }

    public DriveState getDriveState() {
        return driveState;
    }

    public void setDriveState(DriveState newState) {
        driveState = newState;
    }

    public Command setDriveStateCommand(DriveState newState) {
        return this.runOnce(() -> driveState = newState);
    }

    public Command toggleRotationLock() {
        return this.runOnce(() -> rotLock = !rotLock);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (goalState != currentState && !isTransitioning() && atCurrentState() && DriverStation.isEnabled()) {
            switch (goalState) {
                case TravelPosition:
                    currentCommand = new TravelPosition(diff, elevator, this);
                    break;
                case IntakePosition:
                    currentCommand = new IntakePosition(diff, elevator, this);
                    if (currentState != State.TravelPosition) {
                        currentCommand.beforeStarting(new TravelPosition(diff, elevator, this));
                    }
                    currentCommand.andThen(new IntakeCoral(manipulator, this).andThen(setDriveStateCommand(DriveState.Teleop)));
                    break;
                case L1Position:
                    currentCommand = new L1Position(diff, elevator, drive, this);
                    if (currentState == State.IntakePosition) {
                        currentCommand.beforeStarting(new TravelPosition(diff, elevator, this));
                    }
                    break;
                case L2Position:
                    currentCommand = new L2Position(diff, elevator, drive, this);
                    if (currentState == State.IntakePosition) {
                        currentCommand.beforeStarting(new TravelPosition(diff, elevator, this));
                    }
                    break;
                case L3Position:
                    currentCommand = new L3Position(diff, elevator, drive, this);
                    if (currentState == State.IntakePosition) {
                        currentCommand.beforeStarting(new TravelPosition(diff, elevator, this));
                    }
                    break;
                case L4Position:
                    currentCommand = new L4Position(diff, elevator, drive, this);
                    if (currentState == State.IntakePosition) {
                        currentCommand.beforeStarting(new TravelPosition(diff, elevator, this));
                    }
                    break;
                case StartPosition:
                    currentCommand = new TravelPosition(diff, elevator, this);
                    break;
                default:
                    System.out.println("Unknown State!!!!!!!!!!");
                    break;
            }
            currentCommand.schedule();
            transitioning = true;
        }
        stateDash.update(Constants.debugMode);
    }
}
