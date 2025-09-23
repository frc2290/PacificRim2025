// Copyright (c) 2025 FRC 2290
// http://https://github.com/frc2290
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.
//
package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DriveCommands.DriveCommandFactory;
import frc.robot.commands.GraphCommand;
import frc.robot.commands.GraphCommand.GraphCommandNode;
import frc.utils.FlytDashboardV2;
import frc.utils.PoseEstimatorSubsystem;
import java.util.EnumMap;
import java.util.Map;

/** Manages the robot's drive modes using a graph-based state machine. */
public class DriveStateMachine extends SubsystemBase {

  /** Graph helper that orchestrates drive commands as state transitions. */
  private GraphCommand m_graphCommand = new GraphCommand();

  /** Dashboard binding used to visualize the current drive state. */
  private FlytDashboardV2 dashboard = new FlytDashboardV2("DriveStateMachine");

  private DriveSubsystem drive;
  private PoseEstimatorSubsystem pose;
  private XboxController driverController;
  private final DriveCommandFactory driveCommandFactory;
  private double bargeHeadingDegrees;
  private double climbHeadingDegrees;

  /** Enumerates the major driving behaviors the robot can request. */
  public enum DriveState {
    /** Field-oriented manual drive for teleoperated control. */
    MANUAL,
    /** PathPlanner-driven trajectory following. */
    FOLLOW_PATH,
    /** Manual control that keeps the robot pointed toward the barge. */
    BARGE_RELATIVE,
    /** Driver control tuned for endgame climbing. */
    CLIMB_RELATIVE,
    /** Manual control that faces the processor for algae cycles. */
    PROCESSOR_RELATIVE,
    /** Auto-line up with the coral station to intake a game piece. */
    CORAL_STATION,
    /** Field-relative drive that orients around the reef for scoring. */
    REEF_RELATIVE,
    /** Assisted alignment that locks the heading to the active reef branch. */
    REEF_ALIGN,
    /** Idle state when the drivetrain should not accept driver commands. */
    CANCELLED
  }

  /*
   * Graph Command Nodes for Drive State Machine
   */
  /** Quick lookup from a drive state to the graph node that owns its command. */
  private final EnumMap<DriveState, GraphCommandNode> nodes = new EnumMap<>(DriveState.class);

  // State variables.
  /** True when the driver wants to score on the right branch of the reef. */
  private boolean rightScore = false;

  /**
   * Constructor for the DriveStateMachine
   *
   * @param m_drive
   * @param m_pose
   * @param m_driverController
   */
  public DriveStateMachine(
      DriveSubsystem m_drive, PoseEstimatorSubsystem m_pose, XboxController m_driverController) {
    drive = m_drive;
    pose = m_pose;
    driverController = m_driverController;
    bargeHeadingDegrees = pose.getDegrees();
    climbHeadingDegrees = pose.getDegrees();
    driveCommandFactory = new DriveCommandFactory(drive, pose, driverController);

    // Initialize graph command.
    initializeGraphCommand();

    // Set as default command so it runs all the time.
    m_graphCommand.addRequirements(this);
    this.setDefaultCommand(m_graphCommand);
    m_graphCommand.setCurrentNode(nodes.get(DriveState.CANCELLED));
  }

  /**
   * Builds the directed graph that maps each {@link DriveState} to the command that should run
   * while the state is active. Each call to {@link GraphCommandNode#AddNode} defines an allowed
   * transition between states.
   */
  private void initializeGraphCommand() {
    // Create all graph command nodes.
    GraphCommandNode manualNode =
        m_graphCommand
        .new GraphCommandNode(
            "Manual",
            null,
            null,
            // Field-relative manual driving that mirrors the legacy ManualDrive command.
            driveCommandFactory.createManualDriveCommand());
    nodes.put(DriveState.MANUAL, manualNode);

    GraphCommandNode followPathNode =
        m_graphCommand
        .new GraphCommandNode(
            "FollowPath",
            // Autonomous-style follower that keeps path planner targets while honoring rotation
            // overrides.
            driveCommandFactory.createFollowPathCommand(),
            null,
            null);
    nodes.put(DriveState.FOLLOW_PATH, followPathNode);

    GraphCommandNode bargeRelativeNode =
        m_graphCommand
        .new GraphCommandNode(
            "BargeRelative",
            // Capture the driver's current heading when this state becomes active.
            Commands.runOnce(() -> bargeHeadingDegrees = pose.getDegrees()),
            null,
            // Hold the stored heading so the driver can strafe relative to the barge structure.
            driveCommandFactory.createHeadingLockCommand(() -> bargeHeadingDegrees));
    nodes.put(DriveState.BARGE_RELATIVE, bargeRelativeNode);

    GraphCommandNode climbRelativeNode =
        m_graphCommand
        .new GraphCommandNode(
            "ClimbRelative",
            // Snapshot the current heading so the climber keeps its orientation steady.
            Commands.runOnce(() -> climbHeadingDegrees = pose.getDegrees()),
            null,
            // Keep the climber pointed the same direction unless the driver twists the stick.
            driveCommandFactory.createHeadingLockCommand(() -> climbHeadingDegrees));
    nodes.put(DriveState.CLIMB_RELATIVE, climbRelativeNode);

    GraphCommandNode processorRelativeNode =
        m_graphCommand
        .new GraphCommandNode(
            "ProcessorRelative",
            null,
            null,
            // Keep the robot pointed at the processor while letting the driver translate freely.
            driveCommandFactory.createPointingAtPoseCommand(
                () -> VisionConstants.PROCESSOR_AIM_POSE, false));
    nodes.put(DriveState.PROCESSOR_RELATIVE, processorRelativeNode);

    GraphCommandNode coralStationNode =
        m_graphCommand
        .new GraphCommandNode(
            "CoralStation",
            null,
            null,
            // Lock the heading toward whichever coral station is closer based on vision.
            driveCommandFactory.createHeadingLockCommand(
                () ->
                    pose.isClosestStationRight()
                        ? VisionConstants.coralStationRightHeading
                        : VisionConstants.coralStationLeftHeading));
    nodes.put(DriveState.CORAL_STATION, coralStationNode);

    GraphCommandNode reefRelativeNode =
        m_graphCommand
        .new GraphCommandNode(
            "ReefRelative",
            null,
            null,
            // Field-relative driving while staying aimed at the center of the reef.
            driveCommandFactory.createPointingAtPoseCommand(
                () -> VisionConstants.REEF_CENTER_AIM_POSE, false));
    nodes.put(DriveState.REEF_RELATIVE, reefRelativeNode);

    GraphCommandNode reefAlignNode =
        m_graphCommand
        .new GraphCommandNode(
            "ReefAlign",
            null,
            null,
            // Blend manual control with PID corrections that hold the closest reef branch.
            driveCommandFactory.createHoldPoseCommand(
                () -> pose.getClosestBranch(getRightScore()), 0.5, true));
    nodes.put(DriveState.REEF_ALIGN, reefAlignNode);

    GraphCommandNode cancelledNode =
        m_graphCommand
        .new GraphCommandNode(
            "Cancelled",
            null,
            null,
            driveCommandFactory.createCancelledCommand());
    nodes.put(DriveState.CANCELLED, cancelledNode);

    // Graph Command setup.
    m_graphCommand.setGraphRootNode(cancelledNode); // Should not drive.

    // Define transitions between drive states.
    connectAllNodes();
  }

  /** Links every state to every other state so the driver can request any transition. */
  private void connectAllNodes() {
    for (DriveState fromState : DriveState.values()) {
      GraphCommandNode fromNode = nodes.get(fromState);
      if (fromNode == null) {
        continue;
      }

      for (DriveState toState : DriveState.values()) {
        if (fromState == toState) {
          continue;
        }

        GraphCommandNode toNode = nodes.get(toState);
        if (toNode != null) {
          fromNode.AddNode(toNode, 1.0);
        }
      }
    }
  }

  // Branch selection helpers.

  /** Returns true when the driver selected the right reef branch. */
  public boolean getRightScore() {
    return rightScore;
  }

  /** Sets which reef branch the driver wants to score on. */
  public void setRightScore(boolean right) {
    // True means the driver wants to aim for the right branch, while false targets the left.
    rightScore = right;
  }

  // State transition commands.

  /**
   * Requests a new drive state. The underlying {@link GraphCommand} handles whatever transitions
   * are necessary to reach the node safely.
   */
  public void setDriveCommand(DriveState driveState) {
    GraphCommandNode targetNode = nodes.get(driveState);
    if (targetNode == null) {
      targetNode = nodes.get(DriveState.MANUAL);
    }
    m_graphCommand.setTargetNode(targetNode);
  }

  /**
   * @return {@code true} while the graph is still traversing toward the requested state.
   */
  public boolean isTransitioning() {
    return m_graphCommand.isTransitioning();
  }

  /**
   * @return true when the pose estimator reports the drivetrain is at the requested pose.
   */
  public boolean atPosition() {
    return pose.atTargetPose();
  }

  // State getters.

  /** Returns the drive state associated with the current graph node. */
  public DriveState getCurrentState() {
    GraphCommandNode currentNode = m_graphCommand.getCurrentNode();
    for (Map.Entry<DriveState, GraphCommandNode> entry : nodes.entrySet()) {
      if (entry.getValue() == currentNode) {
        return entry.getKey();
      }
    }
    return DriveState.MANUAL; // Default
  }

  // Periodic bookkeeping.
  @Override
  public void periodic() {
    // Update dashboard with the human-friendly state name.
    refreshLatchedHeadings();
    dashboard.putString("Current State", getCurrentState().toString());
    dashboard.putBoolean("At State", !isTransitioning());
    dashboard.putString("Branch", getRightScore() ? "Right" : "Left");
    dashboard.putBoolean("At Drive Position", atPosition());
  }

  /** Updates latched headings whenever the driver rotates the robot manually. */
  private void refreshLatchedHeadings() {
    double manualRot =
        MathUtil.applyDeadband(driverController.getRightX(), OIConstants.kDriveDeadband);
    if (manualRot == 0.0) {
      return;
    }

    switch (getCurrentState()) {
      case BARGE_RELATIVE:
        bargeHeadingDegrees = pose.getDegrees();
        break;
      case CLIMB_RELATIVE:
        climbHeadingDegrees = pose.getDegrees();
        break;
      default:
        break;
    }
  }
}
