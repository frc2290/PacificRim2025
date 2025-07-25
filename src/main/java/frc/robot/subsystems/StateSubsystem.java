// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.IntakeAlgae;
import frc.robot.commands.IntakeCoral;
import frc.robot.commands.Positions.AlgaeL2Position;
import frc.robot.commands.Positions.AlgaeL3Position;
import frc.robot.commands.Positions.ClimbPosition;
import frc.robot.commands.Positions.IntakePosition;
import frc.robot.commands.Positions.L1Position;
import frc.robot.commands.Positions.L2Position;
import frc.robot.commands.Positions.L3Position;
import frc.robot.commands.Positions.L4Position;
import frc.robot.commands.Positions.ProcessorPosition;
import frc.robot.commands.Positions.TravelPosition;
import frc.robot.commands.Waits.SetGoalWait;
import frc.utils.LEDEffects;
import frc.utils.LEDUtility;
import frc.utils.PoseEstimatorSubsystem;
import frc.utils.FLYTLib.FLYTDashboard.FlytLogger;
import frc.utils.LEDEffects.LEDEffect;

public class StateSubsystem extends SubsystemBase {

    private ElevatorSubsystem elevator;
    private DifferentialSubsystem diff;
    private DriveSubsystem drive;
    private ManipulatorSubsystem manipulator;
    private LEDUtility ledUtility;
    private PoseEstimatorSubsystem pose;

    /**
     * Robot State Options - Primarily for elevator and arm
     */
    public enum PositionState {
        TravelPosition,
        IntakePosition,
        L1Position,
        L2Position,
        L3Position,
        L4Position,
        StartPosition,
        ClimbPosition,
        AlgaeL3Position,
        AlgaeL2Position,
        ProcessorPosition,
        Cancelled
    }

    /**
     * Robot Drive States
     */
    public enum DriveState {
        ReefScoreMove,
        ReefScore,
        NetScore,
        ProcessorScore,
        CoralStation,
        Climb,
        Teleop,
        Auto
    }

    /**
     * Manipulator Score States
     */
    public enum ManipulatorState {
        CoralIntake,
        AlgaeIntake,
        HasCoral,
        HasAlgae,
        ScoringCoral,
        ScoringAlgae
    }

    // Rotation lock and current drive state
    private boolean rotLock = true; // TURN BACK TO TRUE
    private DriveState driveState = DriveState.Teleop;

    // Boolean for if we want to score on the left or right branch
    private boolean rightScore = false;

    // State storage
    private PositionState prevState = PositionState.StartPosition;
    private PositionState currentState = PositionState.StartPosition;
    private PositionState goalState = PositionState.StartPosition; //PositionState.TravelPosition;

    // Boolean for if robot is currently transitioning states
    private boolean transitioning = false;

    // Storage for current running command or sequence of commands
    private Command currentCommand = null;

    private boolean isAuto = false;
    private boolean isDisabled = false;

    private FlytLogger stateDash = new FlytLogger("State");

    /** Creates a new StateSubsystem. */
    public StateSubsystem(DifferentialSubsystem m_diff, ElevatorSubsystem m_elevator, DriveSubsystem m_drive, ManipulatorSubsystem m_manip, PoseEstimatorSubsystem m_pose, LEDUtility m_ledUtility) {
        diff = m_diff;
        elevator = m_elevator;
        drive = m_drive;
        manipulator = m_manip;
        ledUtility = m_ledUtility;
        pose = m_pose;

        stateDash.addStringPublisher("Current State", false, () -> getCurrentState().toString());
        stateDash.addStringPublisher("Prev State", false, () -> getPrevState().toString());
        stateDash.addStringPublisher("Goal State", false, () -> getGoalState().toString());
        stateDash.addBoolPublisher("Transitioning", false, () -> isTransitioning());
        stateDash.addBoolPublisher("At State", false, () -> atCurrentState());
        stateDash.addStringPublisher("Drive State", false, () -> getDriveState().toString());
        stateDash.addBoolPublisher("Rotation Lock", false, () -> getRotationLock());
        stateDash.addBoolPublisher("Right Score", false, () -> getRightScore());
    }

    /** Triggers? */
    public Trigger atTarget() {
        return new Trigger(() -> pose.atTargetPose() && atCurrentState());
    }

    public boolean atAlgaePosition() {
        return (currentState == PositionState.AlgaeL2Position || currentState == PositionState.AlgaeL3Position);
    }
    
    public Trigger atAlgaePositionTrigger() {
        return new Trigger(() -> atAlgaePosition());
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
    public PositionState getCurrentState() {
        return currentState;
    }

    public boolean atSafeState() {
        return (getCurrentState() == PositionState.IntakePosition ||
                getCurrentState() == PositionState.StartPosition ||
                getCurrentState() == PositionState.Cancelled);
    }

    /**
     * Get if robot is at current state based on all subsystems being at their setpoint
     * @return True if at current state
     */
    public boolean atCurrentState() {
        return elevator.atPosition() && diff.atExtenstionSetpoint() && diff.atRotationSetpoint() && !isTransitioning();
    }

    /**
     * Get previous state of the robot
     * @return Previous state of the robot
     */
    public PositionState getPrevState() {
        return prevState;
    }

    /**
     * Get goal state of the robot
     * @return Goal state of the robot
     */
    public PositionState getGoalState() {
        return goalState;
    }

    /**
     * Set the current state of the robot. Also sets the previous state to where it was before
     * @param curState State the robot is currently at
     */
    public void setCurrentState(PositionState curState) {
        transitioning = false;
        prevState = currentState;
        currentState = curState;
        System.out.println("Current: " + currentState.toString() + " Prev: " + prevState.toString() + " Goal: "
                + goalState.toString());
    }

    public Command setCurrentStateCommand(PositionState curState) {
        return Commands.runOnce(() -> {
            setCurrentState(curState);
        });
    }

    /**
     * Set the goal state for the robot i.e. where we want it to go
     * @param newState Goal state for the robot to go to
     */
    public void setGoal(PositionState newState) {
        // if (currentCommand != null) {
        //     currentCommand.cancel();
        // }
        goalState = newState;
        transitioning = false;
        System.out.println("New Goal: " + newState.toString());
        // currentState = newState;
    }

    public boolean atGoal() {
        return goalState == currentState;
    }

    /**
     * Cancels current running command or sequence of commands. Also sets current state to cancelled and sets the subsystems to their current position
     */
    public void cancelCurrentCommand() {
        currentCommand.cancel();
        setCurrentState(PositionState.Cancelled);
        //setGoal(PositionState.Cancelled);
        goalState = PositionState.Cancelled;
        elevator.setElevatorSetpoint(elevator.getPosition());
        diff.setExtensionSetpoint(diff.getExtensionPosition());
        diff.setRotationSetpoint(diff.getRotationPosition());
    }

    /**
     * Cancels current running command or sequence of commands. Also sets current state to cancelled and sets the subsystems to their current position
     * @return Instant command to cancel current running command
     */
    public Command cancelCommand() {
        return Commands.runOnce(() -> cancelCurrentCommand());
    }

    /**
     * Set the goal state for the robot i.e. where we want it to go
     * @param newGoal Goal state for the robot to go to
     * @return Instant command to set goal state
     */
    public Command setGoalCommand(PositionState newGoal) {
        return Commands.runOnce(() -> setGoal(newGoal));
    }

    /**
     * Set the goal state for the robot i.e. where we want it to go
     * @param newGoal Goal state for the robot to go to
     * @param wait Boolean to determine if we want to wait for it to finish (True will wait)
     * @return Instant command to set goal state or command with a wait until at goal
     */
    public Command setGoalCommand(PositionState newGoal, boolean wait) {
        if (wait) {
            return new SetGoalWait(this, newGoal);
        } else {
            return setGoalCommand(newGoal);
        }
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
        return Commands.runOnce(() -> setRightScore(right));
    }

    /**
     * Get if robot has coral
     * @return True if has coral
     */
    public boolean hasCoral() {
        return manipulator.hasCoral();
    }

    /**
     * Get if robot has algae
     * @return True if has algae
     */
    public boolean hasAlgae() {
        return manipulator.hasAlgae();
    }

    /**
     * Check if elevator is at a safe height to move
     * @return True if elevator is at a safe height
     */
    public boolean safeToMove() {
        return elevator.getPosition() < 1;
    }

    /** ----- Drive State Section ----- */
    
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
        return Commands.runOnce(() -> setDriveState(newState));
    }

    /**
     * Toggle robot drive rotation lock
     * @return Instant command to toggle rotation lock
     */
    public Command toggleRotationLock() {
        return Commands.runOnce(() -> rotLock = !rotLock);
    }

    /** Other */

    public boolean isAuto() {
        return isAuto;
    }

    public Trigger isAutoTrigger() {
        return new Trigger(() -> isAuto);
    }

    public void setAuto(boolean auto) {
        isAuto = auto;
    }

    public boolean isDisabled() {
        return isDisabled;
    }

    public void setDisabled(boolean disabled) {
        isDisabled = disabled;
    }

    @Override
    public void periodic() {
        /**
         * State management system
         * Check if its at the goal, otherwise run the command needed to reach the goal
         * Inside each case for each state is specific controls to add moves in case it needs to go somewhere else first (Mostly been moved to individual commands)
         * Finally schedules command(s) at the bottom to be executed
         */
        if (goalState != currentState && !isTransitioning() && atCurrentState() && DriverStation.isEnabled()) {
            switch (goalState) {
                case TravelPosition:
                    currentCommand = new TravelPosition(diff, elevator, this);
                    break;
                case IntakePosition:
                    currentCommand = new IntakePosition(diff, elevator, this);
                    currentCommand = currentCommand.andThen(new IntakeCoral(manipulator, this));
                    break;
                case L1Position:
                    currentCommand = new L1Position(diff, elevator, this);
                    //if (currentState == PositionState.IntakePosition) {
                    //    currentCommand = currentCommand.beforeStarting(new TravelPosition(diff, elevator, this));
                    //}
                    break;
                case L2Position:
                    currentCommand = new L2Position(diff, elevator, this);
                    break;
                case L3Position:
                    currentCommand = new L3Position(diff, elevator, this);
                    break;
                case L4Position:
                    currentCommand = new L4Position(diff, elevator, this);
                    break;
                case ClimbPosition:
                    currentCommand = new ClimbPosition(elevator, this);
                    if (currentState != PositionState.TravelPosition) {
                        currentCommand = currentCommand.beforeStarting(new TravelPosition(diff, elevator, this));
                    }
                    break;
                case AlgaeL3Position:
                    currentCommand = new AlgaeL3Position(diff, elevator, this);
                    currentCommand = currentCommand.andThen(new IntakeAlgae(manipulator, this));
                    break;
                case AlgaeL2Position:
                    currentCommand = new AlgaeL2Position(diff, elevator, this);
                    currentCommand = currentCommand.andThen(new IntakeAlgae(manipulator, this));
                    break;
                case ProcessorPosition:
                    currentCommand = new ProcessorPosition(diff, elevator, this);
                    break;
                case StartPosition:
                    //Nothing
                    //currentCommand = new TravelPosition(diff, elevator, this);
                    break;
                default:
                    System.out.println("Unknown State!!!!!!!!!!");
                    break;
            }
            currentCommand.schedule();
            transitioning = true;
        }

        // LED Management?
        if (atCurrentState()) {
            if (isAuto()) {
                ledUtility.setAll(LEDEffect.PULSE, LEDEffects.flytBlue);
            } else if (isDisabled()) {
                ledUtility.getStrip("TopLeft").setEffect(LEDEffect.NAVLIGHTS, Color.kRed);
                ledUtility.getStrip("TopRight").setEffect(LEDEffect.NAVLIGHTS, Color.kGreen);
                ledUtility.getStrip("Left").setEffect(LEDEffect.PULSE, LEDEffects.flytBlue);
                ledUtility.getStrip("Right").setEffect(LEDEffect.PULSE, LEDEffects.flytBlue);
            } else if (getCurrentState() == PositionState.IntakePosition && !manipulator.hasCoral() && getDriveState() == DriveState.CoralStation) {
                ledUtility.getStrip("Left").setEffect(LEDEffect.FLASH, Color.kGreen);
                ledUtility.getStrip("Right").setEffect(LEDEffect.FLASH, Color.kGreen);
            } else if (getCurrentState() == PositionState.IntakePosition && manipulator.hasCoral() && getDriveState() == DriveState.CoralStation) {
                ledUtility.getStrip("Left").setEffect(LEDEffect.SOLID, Color.kGreen);
                ledUtility.getStrip("Right").setEffect(LEDEffect.SOLID, Color.kGreen);
            } else if (!getRotationLock()) {
                ledUtility.getStrip("TopLeft").setEffect(LEDEffect.SOLID, Color.kRed);
                ledUtility.getStrip("TopRight").setEffect(LEDEffect.SOLID, Color.kRed);
                ledUtility.getStrip("Left").setEffect(LEDEffect.PULSE, LEDEffects.flytBlue);
                ledUtility.getStrip("Right").setEffect(LEDEffect.PULSE, LEDEffects.flytBlue);
            } else if (getDriveState() == DriveState.Teleop) {
                if (getRightScore()) {
                    ledUtility.getStrip("TopLeft").setEffect(LEDEffect.SOLID, Color.kPurple);
                    ledUtility.getStrip("TopRight").setEffect(LEDEffect.SOLID, Color.kPurple);
                } else {
                    ledUtility.getStrip("TopLeft").setEffect(LEDEffect.SOLID, Color.kYellow);
                    ledUtility.getStrip("TopRight").setEffect(LEDEffect.SOLID, Color.kYellow);
                }
                ledUtility.getStrip("Left").setEffect(LEDEffect.PULSE, LEDEffects.flytBlue);
                ledUtility.getStrip("Right").setEffect(LEDEffect.PULSE, LEDEffects.flytBlue);
            } else if (getDriveState() == DriveState.ReefScoreMove) {
                ledUtility.getStrip("Left").setEffect(LEDEffect.FLASH, LEDEffects.flytBlue);
                ledUtility.getStrip("Right").setEffect(LEDEffect.FLASH, LEDEffects.flytBlue);
            } else if (getDriveState() == DriveState.ReefScore) {
                ledUtility.getStrip("Left").setEffect(LEDEffect.SOLID, Color.kGreen);
                ledUtility.getStrip("Right").setEffect(LEDEffect.SOLID, Color.kGreen);
            } 
        }

        /** Diff Arm Interpolation */
        if (atInterpolateScoreState() && diff.hasLaserCanDistance() && !isAuto() && manipulator.hasCoral()) {
        //if (atInterpolateScoreState() && diff.hasLaserCanDistance() && manipulator.hasCoral()) {
                if (currentState == PositionState.L4Position) {
                diff.setExtensionSetpoint(diff.l4ExtensionInterpolate());
                diff.setRotationSetpoint(diff.l4RotationInterpolate());
            } else {
                diff.setExtensionSetpoint(diff.l2_3ExtensionInterpolate());
                diff.setRotationSetpoint(diff.l2_3RotationInterpolate());
            }
        }

        stateDash.update(Constants.debugMode);
    }

    private boolean atInterpolateScoreState() {
        return currentState == PositionState.L2Position
                || currentState == PositionState.L3Position || currentState == PositionState.L4Position;
    }
}
