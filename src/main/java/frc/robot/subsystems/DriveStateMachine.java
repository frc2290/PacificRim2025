package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DriveCommands.CancelledDrive;
import frc.robot.commands.DriveCommands.CoralStationDrive;
import frc.robot.commands.DriveCommands.ManualDrive;
import frc.robot.commands.DriveCommands.ProcessorRelativeDrive;
import frc.robot.commands.DriveCommands.ReefAlignDrive;
import frc.robot.commands.DriveCommands.ReefRelativeDrive;
import frc.utils.GraphCommand;
import frc.utils.GraphCommand.GraphCommandNode;
import frc.utils.FlytDashboardV2;
import frc.utils.PoseEstimatorSubsystem;

public class DriveStateMachine extends SubsystemBase {

  private GraphCommand m_graphCommand = new GraphCommand();
  private FlytDashboardV2 dashboard = new FlytDashboardV2("DriveStateMachine");
  private DriveSubsystem drive;
  private PoseEstimatorSubsystem pose;
  private XboxController driverController;

  /** DriveTrain states - drive state machine */
  public enum DriveState {
    MANUAL, // Field oriented freeroam
    FOLLOW_PATH, // Auto path following
    BARGE_RELATIVE, // Faces Barge
    CLIMB_RELATIVE, // Faces Climb
    PROCESSOR_RELATIVE, // Faces Processor
    CORAL_STATION, // Faces Intake based on half field
    REEF_RELATIVE, // Faces Reef based on robot position and angles as drives around
    REEF_ALIGN, // Locked to right or left reef, holding position
    CANCELLED // Drive system cancelled
  }

  /*
   * Graph Command Nodes for Drive State Machine
   */
  GraphCommandNode manualNode;
  GraphCommandNode followPathNode;
  GraphCommandNode bargeRelativeNode;
  GraphCommandNode climbRelativeNode;
  GraphCommandNode processorRelativeNode;
  GraphCommandNode coralStationNode;
  GraphCommandNode reefRelativeNode;
  GraphCommandNode reefAlignNode;
  GraphCommandNode cancelledNode;

  // State variables
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

    // Initialize graph command
    initializeGraphCommand();

    // Set as default command so it runs all the time
    m_graphCommand.addRequirements(this);
    this.setDefaultCommand(m_graphCommand);
    m_graphCommand.setCurrentNode(cancelledNode);
  }

  /** Initialize the graph command with all nodes and connections */
  private void initializeGraphCommand() {
    // Create all graph command nodes
    manualNode =
        m_graphCommand
        .new GraphCommandNode(
            "Manual",
            new PrintCommand(""),
            new PrintCommand(""),
            new ManualDrive(this, drive, pose, driverController));

    // followPathNode = m_graphCommand.new GraphCommandNode(
    //     "FollowPath",
    //     new FollowPathDrive(this, drive, pose, driverController),
    //     new InstantCommand(() -> drive.stop()),
    //     new PrintCommand("Arrived at FollowPath state"));

    // bargeRelativeNode = m_graphCommand.new GraphCommandNode(
    //     "BargeRelative",
    //     new InstantCommand(() -> drive.stop()), // Placeholder command
    //     new InstantCommand(() -> drive.stop()),
    //     new PrintCommand("Arrived at BargeRelative state"));

    // climbRelativeNode = m_graphCommand.new GraphCommandNode(
    //     "ClimbRelative",
    //     new InstantCommand(() -> drive.stop()), // Placeholder command
    //     new InstantCommand(() -> drive.stop()),
    //     new PrintCommand("Arrived at ClimbRelative state"));

    processorRelativeNode =
        m_graphCommand
        .new GraphCommandNode(
            "ProcessorRelative",
            new PrintCommand(""),
            new PrintCommand(""),
            new ProcessorRelativeDrive(this, drive, pose, driverController));

    coralStationNode =
        m_graphCommand
        .new GraphCommandNode(
            "CoralStation",
            new PrintCommand(""),
            new PrintCommand(""),
            new CoralStationDrive(this, drive, pose, driverController));

    reefRelativeNode =
        m_graphCommand
        .new GraphCommandNode(
            "ReefRelative",
            new PrintCommand(""),
            new PrintCommand(""),
            new ReefRelativeDrive(this, drive, pose, driverController));

    reefAlignNode =
        m_graphCommand
        .new GraphCommandNode(
            "ReefAlign",
            new PrintCommand(""),
            new PrintCommand(""),
            new ReefAlignDrive(drive, pose, driverController, this));

    cancelledNode =
        m_graphCommand
        .new GraphCommandNode(
            "Cancelled", new PrintCommand(""), new PrintCommand(""), new CancelledDrive(drive));

    // Graph Command setup
    m_graphCommand.setGraphRootNode(cancelledNode); // shouldn't drive

    // Define transitions between drive states
    cancelledNode.AddNode(manualNode, 1.0);
    cancelledNode.AddNode(processorRelativeNode, 1.0);
    cancelledNode.AddNode(coralStationNode, 1.0);
    cancelledNode.AddNode(reefRelativeNode, 1.0);
    // cancelledNode.AddNode(followPathNode, 1.0);
    // cancelledNode.AddNode(bargeRelativeNode, 1.0);
    // cancelledNode.AddNode(climbRelativeNode, 1.0);
    cancelledNode.AddNode(reefAlignNode, 1.0);
    manualNode.AddNode(cancelledNode, 1.0);
    manualNode.AddNode(processorRelativeNode, 1.0);
    manualNode.AddNode(coralStationNode, 1.0);
    manualNode.AddNode(reefRelativeNode, 1.0);
    // manualNode.AddNode(followPathNode, 1.0);
    // manualNode.AddNode(bargeRelativeNode, 1.0);
    // manualNode.AddNode(climbRelativeNode, 1.0);
    manualNode.AddNode(reefAlignNode, 1.0);
    processorRelativeNode.AddNode(manualNode, 1.0);
    processorRelativeNode.AddNode(cancelledNode, 1.0);
    processorRelativeNode.AddNode(coralStationNode, 1.0);
    processorRelativeNode.AddNode(reefRelativeNode, 1.0);
    // processorRelativeNode.AddNode(followPathNode, 1.0);
    // processorRelativeNode.AddNode(bargeRelativeNode, 1.0);
    // processorRelativeNode.AddNode(climbRelativeNode, 1.0);
    processorRelativeNode.AddNode(reefAlignNode, 1.0);
    coralStationNode.AddNode(manualNode, 1.0);
    coralStationNode.AddNode(cancelledNode, 1.0);
    coralStationNode.AddNode(processorRelativeNode, 1.0);
    coralStationNode.AddNode(reefRelativeNode, 1.0);
    // coralStationNode.AddNode(followPathNode, 1.0);
    // coralStationNode.AddNode(bargeRelativeNode, 1.0);
    // coralStationNode.AddNode(climbRelativeNode, 1.0);
    coralStationNode.AddNode(reefAlignNode, 1.0);
    reefRelativeNode.AddNode(manualNode, 1.0);
    reefRelativeNode.AddNode(cancelledNode, 1.0);
    reefRelativeNode.AddNode(processorRelativeNode, 1.0);
    reefRelativeNode.AddNode(coralStationNode, 1.0);
    // reefRelativeNode.AddNode(followPathNode, 1.0);
    // reefRelativeNode.AddNode(bargeRelativeNode, 1.0);
    // reefRelativeNode.AddNode(climbRelativeNode, 1.0);
    reefRelativeNode.AddNode(reefAlignNode, 1.0);
  }

  /** ----- Branch Selection ----- */

  /**
   * Get Current branch selection
   *
   * @return
   */
  public boolean getRightScore() {
    return rightScore;
  }

  /**
   * Set branch, should done only by the state machine
   *
   * @param right
   */
  public void setRightScore(boolean right) {
    rightScore = right;
  }

  /** ----- State Transition Commands ----- */

  /**
   * Set Goal DriveState for DrimeStateMachine
   *
   * @return
   */
  public void setDriveCommand(DriveState m_driveState) {
    switch (m_driveState) {
      case MANUAL:
        m_graphCommand.setTargetNode(manualNode);
        break;
      case FOLLOW_PATH:
        m_graphCommand.setTargetNode(followPathNode);
        break;
      case BARGE_RELATIVE:
        m_graphCommand.setTargetNode(bargeRelativeNode);
        break;
      case CLIMB_RELATIVE:
        m_graphCommand.setTargetNode(climbRelativeNode);
        break;
      case PROCESSOR_RELATIVE:
        m_graphCommand.setTargetNode(processorRelativeNode);
        break;
      case CORAL_STATION:
        m_graphCommand.setTargetNode(coralStationNode);
        break;
      case REEF_RELATIVE:
        m_graphCommand.setTargetNode(reefRelativeNode);
        break;
      case REEF_ALIGN:
        m_graphCommand.setTargetNode(reefAlignNode);
        break;
      case CANCELLED:
        m_graphCommand.setTargetNode(cancelledNode);
        break;
      default:
        m_graphCommand.setTargetNode(manualNode);
        break;
    }
  }

  /**
   * Check if graph command still reaching goal state
   *
   * @return
   */
  public boolean isTransitioning() {
    return m_graphCommand.isTransitioning();
  }

  /** ----- State Getters ----- */

  /**
   * Get Current state
   *
   * @return
   */
  public DriveState getCurrentState() {
    GraphCommandNode currentNode = m_graphCommand.getCurrentNode();
    if (currentNode == manualNode) return DriveState.MANUAL;
    if (currentNode == followPathNode) return DriveState.FOLLOW_PATH;
    if (currentNode == bargeRelativeNode) return DriveState.BARGE_RELATIVE;
    if (currentNode == climbRelativeNode) return DriveState.CLIMB_RELATIVE;
    if (currentNode == processorRelativeNode) return DriveState.PROCESSOR_RELATIVE;
    if (currentNode == coralStationNode) return DriveState.CORAL_STATION;
    if (currentNode == reefRelativeNode) return DriveState.REEF_RELATIVE;
    if (currentNode == reefAlignNode) return DriveState.REEF_ALIGN;
    if (currentNode == cancelledNode) return DriveState.CANCELLED;
    return DriveState.MANUAL; // Default
  }

  /** ----- Periodic ----- */
  @Override
  public void periodic() {
    // Update dashboard
    dashboard.putString("Current State", getCurrentState().toString());
    dashboard.putBoolean("At State", !isTransitioning());
    dashboard.putString("Branch", getRightScore() ? "Right" : "Left");
  }
}
