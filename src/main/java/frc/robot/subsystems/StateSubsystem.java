// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

    /**
     * Robot State Options - Primarily for elevator and arm
     */
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

    /**
     * Robot Drive States
     */
    public enum DriveState {
        ReefScore,
        NetScore,
        ProcessorScore,
        CoralStation,
        Climb,
        Teleop
    }

    // Rotation lock and current drive state
    private boolean rotLock = true;
    private DriveState driveState = DriveState.Teleop;

    // Boolean for if we want to score on the left or right branch
    private boolean rightScore = false;

    // State storage
    private State prevState = State.StartPosition;
    private State currentState = State.StartPosition;
    private State goalState = State.TravelPosition;

    // Boolean for if robot is currently transitioning states
    private boolean transitioning = false;

    // Storage for current running command or sequence of commands
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
        stateDash.addBoolPublisher("Right Score", false, () -> getRightScore());
    }

    /** Robot State Section */

    /**
     * Get if robot is transitioning between states
     * @return True if transitioning
     */
    public boolean isTransitioning() {
        return transitioning;
    }

    /**
     * Get current state robot is in
     * @return Current state of the robot
     */
    public State getCurrentState() {
        return currentState;
    }

    /**
     * Get if robot is at current state based on all subsystems being at their setpoint
     * @return True if at current state
     */
    public boolean atCurrentState() {
        return elevator.atPosition() && diff.atExtenstionSetpoint() && diff.atRotationSetpoint();
    }

    /**
     * Get previous state of the robot
     * @return Previous state of the robot
     */
    public State getPrevState() {
        return prevState;
    }

    /**
     * Get goal state of the robot
     * @return Goal state of the robot
     */
    public State getGoalState() {
        return goalState;
    }

    /**
     * Set the current state of the robot. Also sets the previous state to where it was before
     * @param curState State the robot is currently at
     */
    public void setCurrentState(State curState) {
        transitioning = false;
        prevState = currentState;
        currentState = curState;
        System.out.println("Current: " + currentState.toString() + " Prev: " + prevState.toString() + " Goal: "
                + goalState.toString());
    }

    /**
     * Set the goal state for the robot i.e. where we want it to go
     * @param newState Goal state for the robot to go to
     */
    public void setGoal(State newState) {
        goalState = newState;
        System.out.println("New Goal: " + newState.toString());
        // currentState = newState;
    }

    /**
     * Cancels current running command or sequence of commands. Also sets current state to cancelled and sets the subsystems to their current position
     */
    public void cancelCurrentCommand() {
        currentCommand.cancel();
        setCurrentState(State.Cancelled);
        setGoal(State.Cancelled);
        elevator.setElevatorSetpoint(elevator.getPosition());
        diff.setExtensionSetpoint(diff.getExtensionPosition());
        diff.setRotationSetpoint(diff.getRotationPosition());
    }

    /**
     * Cancels current running command or sequence of commands. Also sets current state to cancelled and sets the subsystems to their current position
     * @return Instant command to cancel current running command
     */
    public Command cancelCommand() {
        return this.runOnce(() -> cancelCurrentCommand());
    }

    /**
     * Set the goal state for the robot i.e. where we want it to go
     * @param newGoal Goal state for the robot to go to
     * @return Instant command to set goal state
     */
    public Command setGoalCommand(State newGoal) {
        return this.runOnce(() -> setGoal(newGoal));
    }

    /**
     * Get if robot wants to score on right branch
     * @return True if right branch
     */
    public boolean getRightScore() {
        return rightScore;
    }

    /**
     * Set if robot wants to score on right branch
     * @param right True if right branch
     */
    public void setRightScore(boolean right) {
        rightScore = right;
    }

    /**
     * Set if robot wants to score on right branch
     * @param right True if right branch
     * @return Instant command to set right branch
     */
    public Command setRightScoreCommand(boolean right) {
        return this.runOnce(() -> setRightScore(right));
    }

    /**
     * Get if robot has coral
     * @return True if has coral
     */
    public boolean hasCoral() {
        return manipulator.hasCoral();
    }

    /**
     * Check if elevator is at a safe height to move
     * @return True if elevator is at a safe height
     */
    public boolean safeToMove() {
        return elevator.getPosition() < 1;
    }

    /** Drive State Section */
    
    /**
     * Get if robot drive is rotation locked
     * @return True if rotation locked
     */
    public boolean getRotationLock() {
        return rotLock;
    }

    /**
     * Set robot drive rotation lock
     * @param lock True if locked
     */
    public void setRotationLock(boolean lock) {
        rotLock = lock;
    }

    /**
     * Get current robot drive state
     * @return Current robot drive state
     */
    public DriveState getDriveState() {
        return driveState;
    }

    /**
     * Set current robot drive state
     * @param newState New robot drive state
     */
    public void setDriveState(DriveState newState) {
        driveState = newState;
    }

    /**
     * Set current robot drive state
     * @param newState New robot drive state
     * @return Instant command to set robot drive state
     */
    public Command setDriveStateCommand(DriveState newState) {
        return this.runOnce(() -> driveState = newState);
    }

    /**
     * Toggle robot drive rotation lock
     * @return Instant command to toggle rotation lock
     */
    public Command toggleRotationLock() {
        return this.runOnce(() -> rotLock = !rotLock);
    }

    @Override
    public void periodic() {
        /**
         * State management system
         * Check if its at the goal, otherwise run the command needed to reach the goal
         * Inside each case for each state is specific controls to add moves in case it needs to go somewhere else first
         * Finally schedules command(s) at the bottom to be executed
         */
        if (goalState != currentState && !isTransitioning() && atCurrentState() && DriverStation.isEnabled()) {
            switch (goalState) {
                case TravelPosition:
                    currentCommand = new TravelPosition(diff, elevator, this);
                    break;
                case IntakePosition:
                    currentCommand = new IntakePosition(diff, elevator, this);
                    if (currentState != State.TravelPosition) {
                        currentCommand = currentCommand.beforeStarting(new TravelPosition(diff, elevator, this));
                    }
                    currentCommand = currentCommand.andThen(new IntakeCoral(manipulator, this));
                    break;
                case L1Position:
                    currentCommand = new L1Position(diff, elevator, drive, this);
                    if (currentState == State.IntakePosition) {
                        currentCommand = currentCommand.beforeStarting(new TravelPosition(diff, elevator, this));
                    }
                    break;
                case L2Position:
                    currentCommand = new L2Position(diff, elevator, drive, this);
                    if (currentState == State.IntakePosition) {
                        currentCommand = currentCommand.beforeStarting(new TravelPosition(diff, elevator, this));
                    }
                    break;
                case L3Position:
                    currentCommand = new L3Position(diff, elevator, drive, this);
                    if (currentState == State.IntakePosition) {
                        currentCommand = currentCommand.beforeStarting(new TravelPosition(diff, elevator, this));
                    }
                    break;
                case L4Position:
                    currentCommand = new L4Position(diff, elevator, drive, this);
                    if (currentState == State.IntakePosition) {
                        currentCommand = currentCommand.beforeStarting(new TravelPosition(diff, elevator, this));
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
