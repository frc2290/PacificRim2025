// Copyright (c) 2025 FRC 2290
// http://https://github.com/frc2290
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorManipulatorPositions;
import frc.robot.commands.ElevatorManipulator.ManipulatorPositionCommandFactory;
import frc.robot.commands.EndEffector.ManipulatorIntakeCoral;
import frc.robot.commands.EndEffector.ScoreCoral;
import frc.robot.commands.GraphCommand;
import frc.robot.commands.GraphCommand.GraphCommandNode;
import frc.utils.FlytDashboardV2;
import frc.utils.PoseEstimatorSubsystem;

/** Coordinates the manipulator, elevator, and climb states via a graph of commands. */
public class ManipulatorStateMachine extends SubsystemBase {

  /** Graph helper that sequences manipulator poses and end-effector commands. */
  private GraphCommand m_graphCommand = new GraphCommand();

  /** Dashboard binding used to monitor manipulator state transitions. */
  private FlytDashboardV2 dashboard = new FlytDashboardV2("ManipulatorStateMachine");

  private DriveSubsystem drive;
  private PoseEstimatorSubsystem pose;
  private XboxController driverController;
  private ElevatorSubsystem m_elevator;
  private DifferentialSubsystem m_diff;
  private ManipulatorSubsystem m_manipulator;
  private ClimbSubsystem m_climb;

  /** Manipulator and Elevator states - Manipulator State Machine */
  public enum ElevatorManipulatorState {
    START_POSITION,
    SAFE_CORAL_TRAVEL,
    INTAKE_CORAL,
    L1,
    L2,
    L3,
    L4,
    ALGAE_L2,
    ALGAE_L3,
    PROCESSOR,
    BARGE,
    CLIMB_READY,
    CLIMBED,
    CLIMB_ABORT,
    MANUAL,
    RESET,

    PrepL1,
    PrepL2,
    PrepL3,
    PrepL4,

    PostL1,
    PostL2,
    PostL3,
    PostL4,
  }

  /*
   * Graph Command Nodes for Drive State Machine
   */
  GraphCommandNode startPositionNode;
  GraphCommandNode intakeCoralNode;
  GraphCommandNode safeCoralTravelNode;
  GraphCommandNode l1PrepNode;
  GraphCommandNode l2PrepNode;
  GraphCommandNode l3PrepNode;
  GraphCommandNode l4PrepNode;
  GraphCommandNode scoreL1Node;
  GraphCommandNode scoreL2Node;
  GraphCommandNode scoreL3Node;
  GraphCommandNode scoreL4Node;
  GraphCommandNode l1PostScoreNode;
  GraphCommandNode l2PostScoreNode;
  GraphCommandNode l3PostScoreNode;
  GraphCommandNode l4PostScoreNode;
  GraphCommandNode prepAlgaeIntakeNode;
  GraphCommandNode prepAlgaeLowNode;
  GraphCommandNode prepAlgaeHighNode;
  GraphCommandNode safeAlgaeTravelNode;
  GraphCommandNode scoreProcessorNode;
  GraphCommandNode prepScoreBargeNode;
  GraphCommandNode scoreBargeNode;
  GraphCommandNode bargePostScoreNode;
  GraphCommandNode climbReadyNode;
  GraphCommandNode climbedNode;
  GraphCommandNode climbAbortNode;
  GraphCommandNode cancelledNode;

  // variables
  /** Tracks whether the current state has reached its goal pose. */
  private boolean atGoalState = false; // checks if it reached the final state

  /** True when the state machine has armed the manipulator to score. */
  private boolean canScore = false; // checks final state is reached and command aproves the score

  /** Driver request that tells the state machine to begin the scoring routine. */
  private boolean score = false; // drive sends command to score

  /** Enables interpolation based on laser distance when targeting certain reef levels. */
  private boolean interpolate = false;

  /** Flag that mirrors whether the drive subsystem is staged at its target pose. */
  private boolean driveAtPose = false;

  private boolean reachGoalStateFailed = false;

  /** Timestamp used to time-out states that take too long. */
  private double stateEntryTime = 0.0;

  /**
   * ManipulatorStateMachine Constructor
   *
   * @param diff
   * @param elevator
   * @param manipulator
   * @param climb
   */
  public ManipulatorStateMachine(
      DifferentialSubsystem diff,
      ElevatorSubsystem elevator,
      ManipulatorSubsystem manipulator,
      ClimbSubsystem climb) {
    m_diff = diff;
    m_elevator = elevator;
    m_manipulator = manipulator;
    m_climb = climb;

    initializeGraphCommand();

    // Set the root/current BEFORE scheduling it as default
    m_graphCommand.setGraphRootNode(startPositionNode);
    m_graphCommand.setCurrentNode(startPositionNode);

    // Now register requirements and set default
    m_graphCommand.addRequirements(this);
    this.setDefaultCommand(m_graphCommand);
  }

  /**
   * Builds the manipulator graph, mapping each logical state to the commands required to reach it
   * safely. The graph allows the state machine to jump between presets without violating joint
   * limits.
   */
  private void initializeGraphCommand() {
    // Build every node in the manipulator graph along with the command to run when
    // we visit it.

    startPositionNode =
        m_graphCommand
        .new GraphCommandNode(
            "StartPosition",
            ManipulatorPositionCommandFactory.createSafeReturnCommand(
                this, m_diff, m_elevator, ElevatorManipulatorPositions.START_POSITION),
            new PrintCommand(""),
            new PrintCommand(""));

    intakeCoralNode =
        m_graphCommand
        .new GraphCommandNode(
            "IntakeCoral",
            ManipulatorPositionCommandFactory.createPrepCommand(
                this, m_diff, m_elevator, ElevatorManipulatorPositions.INTAKE_CORAL),
            new PrintCommand(""),
            new ManipulatorIntakeCoral(m_manipulator));

    safeCoralTravelNode =
        m_graphCommand
        .new GraphCommandNode(
            "SafeCoralTravelPos",
            ManipulatorPositionCommandFactory.createSafeReturnCommand(
                this, m_diff, m_elevator, ElevatorManipulatorPositions.CORAL_TRANSPORT),
            new PrintCommand(""),
            new PrintCommand(""));

    l1PrepNode =
        m_graphCommand
        .new GraphCommandNode(
            "L1Prep",
            ManipulatorPositionCommandFactory.createPrepCommand(
                this, m_diff, m_elevator, ElevatorManipulatorPositions.L1_PREP),
            new PrintCommand(""),
            new PrintCommand(""));

    l2PrepNode =
        m_graphCommand
        .new GraphCommandNode(
            "L2Prep",
            ManipulatorPositionCommandFactory.createPrepCommand(
                this, m_diff, m_elevator, ElevatorManipulatorPositions.L2_PREP),
            new PrintCommand(""),
            new PrintCommand(""));

    l3PrepNode =
        m_graphCommand
        .new GraphCommandNode(
            "L3Prep",
            ManipulatorPositionCommandFactory.createPrepCommand(
                this, m_diff, m_elevator, ElevatorManipulatorPositions.L3_PREP),
            new PrintCommand(""),
            new PrintCommand(""));

    l4PrepNode =
        m_graphCommand
        .new GraphCommandNode(
            "L4Prep",
            ManipulatorPositionCommandFactory.createPrepCommand(
                this, m_diff, m_elevator, ElevatorManipulatorPositions.L4_PREP),
            new PrintCommand(""),
            new PrintCommand(""));

    scoreL1Node =
        m_graphCommand
        .new GraphCommandNode(
            "ScoreL1",
            ManipulatorPositionCommandFactory.createScoreCommand(
                this, m_diff, m_elevator, ElevatorManipulatorPositions.SCORE_L1),
            new PrintCommand(""),
            new ScoreCoral(this, m_manipulator));

    scoreL2Node =
        m_graphCommand
        .new GraphCommandNode(
            "ScoreL2",
            ManipulatorPositionCommandFactory.createScoreCommand(
                this, m_diff, m_elevator, ElevatorManipulatorPositions.SCORE_L2),
            new PrintCommand(""),
            new ScoreCoral(this, m_manipulator));

    scoreL3Node =
        m_graphCommand
        .new GraphCommandNode(
            "ScoreL3",
            ManipulatorPositionCommandFactory.createScoreCommand(
                this, m_diff, m_elevator, ElevatorManipulatorPositions.SCORE_L3),
            new PrintCommand(""),
            new ScoreCoral(this, m_manipulator));

    scoreL4Node =
        m_graphCommand
        .new GraphCommandNode(
            "ScoreL4",
            ManipulatorPositionCommandFactory.createScoreCommand(
                this, m_diff, m_elevator, ElevatorManipulatorPositions.SCORE_L4),
            new PrintCommand(""),
            new ScoreCoral(this, m_manipulator));

    l1PostScoreNode =
        m_graphCommand
        .new GraphCommandNode(
            "L1PostScore",
            ManipulatorPositionCommandFactory.createSafeReturnCommand(
                this, m_diff, m_elevator, ElevatorManipulatorPositions.L1_POST_SCORE),
            new PrintCommand(""),
            new PrintCommand(""));

    l2PostScoreNode =
        m_graphCommand
        .new GraphCommandNode(
            "L2PostScore",
            ManipulatorPositionCommandFactory.createSafeReturnCommand(
                this, m_diff, m_elevator, ElevatorManipulatorPositions.L2_POST_SCORE),
            new PrintCommand(""),
            new PrintCommand(""));

    l3PostScoreNode =
        m_graphCommand
        .new GraphCommandNode(
            "L3PostScore",
            ManipulatorPositionCommandFactory.createSafeReturnCommand(
                this, m_diff, m_elevator, ElevatorManipulatorPositions.L3_POST_SCORE),
            new PrintCommand(""),
            new PrintCommand(""));

    l4PostScoreNode =
        m_graphCommand
        .new GraphCommandNode(
            "L4PostScore",
            ManipulatorPositionCommandFactory.createSafeReturnCommand(
                this, m_diff, m_elevator, ElevatorManipulatorPositions.L4_POST_SCORE),
            new PrintCommand(""),
            new PrintCommand(""));

    prepAlgaeIntakeNode =
        m_graphCommand
        .new GraphCommandNode(
            "PrepAlgaeIntake", new PrintCommand(""), new PrintCommand(""), new PrintCommand(""));

    prepAlgaeLowNode =
        m_graphCommand
        .new GraphCommandNode(
            "PrepAlgaeLow",
            ManipulatorPositionCommandFactory.createPrepCommand(
                this, m_diff, m_elevator, ElevatorManipulatorPositions.PREP_ALGAE_LOW),
            new PrintCommand(""),
            new PrintCommand(""));

    prepAlgaeHighNode =
        m_graphCommand
        .new GraphCommandNode(
            "PrepAlgaeHigh",
            ManipulatorPositionCommandFactory.createPrepCommand(
                this, m_diff, m_elevator, ElevatorManipulatorPositions.PREP_ALGAE_HIGH),
            new PrintCommand(""),
            new PrintCommand(""));

    safeAlgaeTravelNode =
        m_graphCommand
        .new GraphCommandNode(
            "SafeAlgaeTravelPos",
            ManipulatorPositionCommandFactory.createSafeReturnCommand(
                this, m_diff, m_elevator, ElevatorManipulatorPositions.ALGAE_TRANSPORT),
            new PrintCommand(""),
            new PrintCommand(""));

    scoreProcessorNode =
        m_graphCommand
        .new GraphCommandNode(
            "ScoreProcessor",
            ManipulatorPositionCommandFactory.createScoreCommand(
                this, m_diff, m_elevator, ElevatorManipulatorPositions.SCORE_PROCESSOR),
            new PrintCommand(""),
            new PrintCommand(""));

    prepScoreBargeNode =
        m_graphCommand
        .new GraphCommandNode(
            "PrepScoreBarge",
            ManipulatorPositionCommandFactory.createPrepCommand(
                this, m_diff, m_elevator, ElevatorManipulatorPositions.PREP_BARGE),
            new PrintCommand(""),
            new PrintCommand(""));

    scoreBargeNode =
        m_graphCommand
        .new GraphCommandNode(
            "ScoreBarge",
            ManipulatorPositionCommandFactory.createScoreCommand(
                this, m_diff, m_elevator, ElevatorManipulatorPositions.SCORE_BARGE),
            new PrintCommand(""),
            new PrintCommand(""));

    bargePostScoreNode =
        m_graphCommand
        .new GraphCommandNode(
            "BargePostScore",
            ManipulatorPositionCommandFactory.createSafeReturnCommand(
                this, m_diff, m_elevator, ElevatorManipulatorPositions.BARGE_POST_SCORE),
            new PrintCommand(""),
            new PrintCommand(""));

    climbReadyNode =
        m_graphCommand
        .new GraphCommandNode(
            "ClimbReady",
            Commands.sequence(
                ManipulatorPositionCommandFactory.createPrepCommand(
                    this, m_diff, m_elevator, ElevatorManipulatorPositions.CLIMB),
                Commands.runOnce(() -> m_climb.setServoOpen(), m_climb)),
            new PrintCommand(""),
            new PrintCommand(""));

    climbedNode =
        m_graphCommand
        .new GraphCommandNode(
            "Climbed", new PrintCommand(""), new PrintCommand(""), new PrintCommand(""));

    climbAbortNode =
        m_graphCommand
        .new GraphCommandNode(
            "ClimbAbort",
            Commands.sequence(
                Commands.runOnce(() -> m_climb.setServoClose(), m_climb),
                ManipulatorPositionCommandFactory.createSafeReturnCommand(
                    this, m_diff, m_elevator, ElevatorManipulatorPositions.CORAL_TRANSPORT)),
            new PrintCommand(""),
            new PrintCommand(""));

    cancelledNode =
        m_graphCommand
        .new GraphCommandNode(
            "Cancelled", new PrintCommand(""), new PrintCommand(""), new PrintCommand(""));

    // Safe travel connections
    startPositionNode.AddNode(safeCoralTravelNode, 1.0); // start position to safe travel
    safeCoralTravelNode.AddNode(intakeCoralNode, 1.0); // safe travel to coral intake
    safeCoralTravelNode.AddNode(l1PrepNode, 1.0); // safe travel to l1 prep
    safeCoralTravelNode.AddNode(l2PrepNode, 1.0); // safe travel to l2 prep
    safeCoralTravelNode.AddNode(l3PrepNode, 1.0); // safe travel to l3 prep
    safeCoralTravelNode.AddNode(l4PrepNode, 1.0); // safe travel to l4 prep
    intakeCoralNode.AddNode(safeCoralTravelNode, 1.0); // coral intake to safe travel
    l1PrepNode.AddNode(l2PrepNode, 1.0); // l1 prep to l2 prep
    l1PrepNode.AddNode(l3PrepNode, 1.0); // l1 prep to l3 prep
    l1PrepNode.AddNode(l4PrepNode, 1.0); // l1 prep to l4 prep
    l2PrepNode.AddNode(l1PrepNode, 1.0); // l2 prep to l1 prep
    l2PrepNode.AddNode(l3PrepNode, 1.0); // l2 prep to l3 prep
    l2PrepNode.AddNode(l4PrepNode, 1.0); // l2 prep to l4 prep
    l3PrepNode.AddNode(l1PrepNode, 1.0); // l3 prep to l1 prep
    l3PrepNode.AddNode(l2PrepNode, 1.0); // l3 prep to l2 prep
    l3PrepNode.AddNode(l4PrepNode, 1.0); // l3 prep to l4 prep
    l4PrepNode.AddNode(l1PrepNode, 1.0); // l4 prep to l1 prep
    l4PrepNode.AddNode(l2PrepNode, 1.0); // l4 prep to l2 prep
    l4PrepNode.AddNode(l3PrepNode, 1.0); // l4 prep to l3 prep
    l1PrepNode.AddNode(scoreL1Node, 1.0); // l1 prep to score l1
    l2PrepNode.AddNode(scoreL2Node, 1.0); // l2 prep to score l2
    l3PrepNode.AddNode(scoreL3Node, 1.0); // l3 prep to score l3
    l4PrepNode.AddNode(scoreL4Node, 1.0); // l4 prep to score l4
    scoreL1Node.AddNode(l1PrepNode, 1.0); // score l1 to l1 prep
    scoreL2Node.AddNode(l2PrepNode, 1.0); // score l2 to l2 prep
    scoreL3Node.AddNode(l3PrepNode, 1.0); // score l3 to l3 prep
    scoreL4Node.AddNode(l4PrepNode, 1.0); // score l4 to l4 prep
    scoreL1Node.AddNode(l1PostScoreNode, 1.0); // score l1 to l1 post score
    scoreL2Node.AddNode(l2PostScoreNode, 1.0); // score l2 to l2 post score
    scoreL3Node.AddNode(l3PostScoreNode, 1.0); // score l3 to l3 post score
    scoreL4Node.AddNode(l4PostScoreNode, 1.0); // score l4 to l4 post score
    l1PostScoreNode.AddNode(safeCoralTravelNode, 1.0); // l1 post score to safe travel
    l2PostScoreNode.AddNode(safeCoralTravelNode, 1.0); // l2 post score to safe travel
    l3PostScoreNode.AddNode(safeCoralTravelNode, 1.0); // l3 post score to safe travel
    l4PostScoreNode.AddNode(safeCoralTravelNode, 1.0); // l4 post score to safe travel
    safeCoralTravelNode.AddNode(prepAlgaeLowNode, 1.0); // safe travel to prep algae low
    safeCoralTravelNode.AddNode(prepAlgaeHighNode, 1.0); // safe travel to prep algae high
    prepAlgaeLowNode.AddNode(prepAlgaeHighNode, 1.0); // prep algae low to prep algae high
    prepAlgaeHighNode.AddNode(prepAlgaeLowNode, 1.0); // prep algae high to prep algae low
    prepAlgaeLowNode.AddNode(prepAlgaeIntakeNode, 1.0); // prep algae low to prep algae intake
    prepAlgaeHighNode.AddNode(prepAlgaeIntakeNode, 1.0); // prep algae high to prep algae intake
    prepAlgaeIntakeNode.AddNode(prepAlgaeLowNode, 1.0); // prep algae intake to prep algae low
    prepAlgaeIntakeNode.AddNode(prepAlgaeHighNode, 1.0); // prep algae intake to prep algae high
    prepAlgaeLowNode.AddNode(safeAlgaeTravelNode, 1.0); // prep algae low to safe algae travel
    prepAlgaeHighNode.AddNode(safeAlgaeTravelNode, 1.0); // prep algae high to safe algae travel
    safeAlgaeTravelNode.AddNode(scoreProcessorNode, 1.0); // safe algae travel to score processor
    safeAlgaeTravelNode.AddNode(prepScoreBargeNode, 1.0); // safe algae travel to prep score barge
    prepScoreBargeNode.AddNode(scoreBargeNode, 1.0); // prep score barge to score barge
    scoreBargeNode.AddNode(bargePostScoreNode, 1.0); // score barge to barge post score
    scoreProcessorNode.AddNode(safeCoralTravelNode, 1.0); // score processor to safe coral travel
    bargePostScoreNode.AddNode(safeCoralTravelNode, 1.0); // barge post score to safe coral travel
    safeCoralTravelNode.AddNode(climbReadyNode, 1.0); // safe coral travel to climb ready
    climbReadyNode.AddNode(climbedNode, 1.0); // climb ready to climbed
    climbReadyNode.AddNode(climbAbortNode, 1.0); // climb ready to climb abort
    climbAbortNode.AddNode(safeCoralTravelNode, 1.0); // climb abort to safe coral travel
  }

  // need for scoring, make sure drive subsystem is at pose before scoring
  /** Called by the drive state machine to indicate whether the chassis is aligned to score. */
  public void setDriveAtPose(boolean atPose) {
    driveAtPose = atPose;
  }

  public void setInterpolation(boolean m_interpolate) {
    interpolate = m_interpolate;
  }

  /**
   * Tell state machine that command finished succesfully (Shuold only be set by commands)
   *
   * @param state
   */
  public void setatGoalState(boolean state) {
    atGoalState = state;
  }

  // Backwards-compatible setter used by existing commands
  public void atGoalState(boolean state) {
    setatGoalState(state);
  }

  // should only be used by commands, lets scoring command know that systems are
  // ready to score
  public void setreadyToScore(boolean canscore) {
    canScore = canscore;
  }

  /**
   * Can only be used by commands, tell statemachine that command too to long to complete, needs
   * reset, or revert to prep node
   */
  public void failedToReachGoal(boolean isFailed) {
    reachGoalStateFailed = isFailed;
  }

  // used by a coardinator to tell statemachine that it needs to score if it can
  public void score(boolean m_score) {
    score = m_score;
  }

  public boolean getInterpolateActive() {
    return interpolate;
  }

  public boolean getHasCoral() {
    return m_manipulator.hasCoral();
  }

  public boolean readyToScore() {
    return canScore;
  }

  // driver made a call to score now
  public boolean scoreNow() {
    return score;
  }

  /**
   * GraphCommand is still reaching setgoalnode
   *
   * @return
   */
  public boolean isTransitioning() {
    return m_graphCommand.isTransitioning();
  }

  private GraphCommandNode getGraphState() {
    return m_graphCommand.getCurrentNode();
  }

  /**
   * Check if all all subsystem are at position, atGoalState is updated by a last command
   *
   * @return
   */
  public boolean atGoalState() {
    return atGoalState;
  }

  public Command waitUntilReady() {
    return Commands.waitUntil(() -> atGoalState() && !isTransitioning());
  }

  public boolean reachGoalStateFailed() {
    return reachGoalStateFailed;
  }

  public Command waitForState(ElevatorManipulatorState desiredState) {
    return Commands.waitUntil(() -> getCurrentState() == desiredState && !isTransitioning());
  }

  /** ----- State Transition Commands ----- */

  /**
   * Sets the goal state for the manipulator state machine
   *
   * @param m_state
   */
  public void setElevatorManipulatorCommand(ElevatorManipulatorState m_state) {
    switch (m_state) {
      case SAFE_CORAL_TRAVEL:
        m_graphCommand.setTargetNode(safeCoralTravelNode);
        break;
      case INTAKE_CORAL:
        m_graphCommand.setTargetNode(intakeCoralNode);
        break;
      case L1:
        m_graphCommand.setTargetNode(scoreL1Node);
        break;
      case L2:
        m_graphCommand.setTargetNode(scoreL2Node);
        break;
      case L3:
        m_graphCommand.setTargetNode(scoreL3Node);
        break;
      case L4:
        m_graphCommand.setTargetNode(scoreL4Node);
        break;
      case ALGAE_L2:
        m_graphCommand.setTargetNode(prepAlgaeLowNode);
        break;
      case ALGAE_L3:
        m_graphCommand.setTargetNode(prepAlgaeHighNode);
        break;
      case PROCESSOR:
        m_graphCommand.setTargetNode(scoreProcessorNode);
        break;
      case BARGE:
        m_graphCommand.setTargetNode(scoreBargeNode);
        break;
      case CLIMB_READY:
        m_graphCommand.setTargetNode(climbReadyNode);
        break;
      case CLIMBED:
        m_graphCommand.setTargetNode(climbedNode);
        break;
      case CLIMB_ABORT:
        m_graphCommand.setTargetNode(climbAbortNode);
        break;
      case MANUAL:
        break;
      default:
        break;
    }
  }

  /**
   * Return current state
   *
   * @return
   */
  public ElevatorManipulatorState getCurrentState() {
    GraphCommandNode currentNode = m_graphCommand.getCurrentNode();
    if (currentNode == startPositionNode) return ElevatorManipulatorState.START_POSITION;
    if (currentNode == safeCoralTravelNode) return ElevatorManipulatorState.SAFE_CORAL_TRAVEL;
    if (currentNode == intakeCoralNode) return ElevatorManipulatorState.INTAKE_CORAL;
    if (currentNode == l1PrepNode) return ElevatorManipulatorState.PrepL1;
    if (currentNode == scoreL1Node) return ElevatorManipulatorState.L1;
    if (currentNode == l1PostScoreNode) return ElevatorManipulatorState.PostL1;
    if (currentNode == l2PrepNode) return ElevatorManipulatorState.PrepL2;
    if (currentNode == scoreL2Node) return ElevatorManipulatorState.L2;
    if (currentNode == l2PostScoreNode) return ElevatorManipulatorState.PostL2;
    if (currentNode == l3PrepNode) return ElevatorManipulatorState.PrepL3;
    if (currentNode == scoreL3Node) return ElevatorManipulatorState.L3;
    if (currentNode == l3PostScoreNode) return ElevatorManipulatorState.PostL3;
    if (currentNode == l4PrepNode) return ElevatorManipulatorState.PrepL4;
    if (currentNode == scoreL4Node) return ElevatorManipulatorState.L4;
    if (currentNode == l4PostScoreNode) return ElevatorManipulatorState.PostL4;
    if (currentNode == prepAlgaeLowNode) return ElevatorManipulatorState.ALGAE_L2;
    if (currentNode == prepAlgaeHighNode) return ElevatorManipulatorState.ALGAE_L3;
    if (currentNode == scoreProcessorNode) return ElevatorManipulatorState.PROCESSOR;
    if (currentNode == prepScoreBargeNode
        || currentNode == scoreBargeNode
        || currentNode == bargePostScoreNode) return ElevatorManipulatorState.BARGE;
    if (currentNode == climbReadyNode) return ElevatorManipulatorState.CLIMB_READY;
    if (currentNode == climbedNode) return ElevatorManipulatorState.CLIMBED;
    if (currentNode == climbAbortNode) return ElevatorManipulatorState.CLIMB_ABORT;
    if (currentNode == cancelledNode) return ElevatorManipulatorState.RESET;
    return ElevatorManipulatorState.MANUAL;
  }

  /** ----- Periodic ----- */
  @Override
  public void periodic() {
    // Update dashboard
    dashboard.putString("Goal State", getCurrentState().toString());
    dashboard.putBoolean("Transitioning", isTransitioning());
    dashboard.putBoolean("DriveTrainLocked", atGoalState());
    dashboard.putString("Current GraphState State", getGraphState().toString());
    dashboard.putBoolean("At state graphcommand", !isTransitioning());
  }
}
