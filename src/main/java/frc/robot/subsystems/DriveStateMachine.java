package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.GraphCommand;
import frc.robot.commands.GraphCommand.GraphCommandNode;
import frc.robot.commands.DriveCommands.DriveCommandFactory;
import frc.utils.FlytDashboardV2;
import frc.utils.PoseEstimatorSubsystem;

/** Manages the robot's drive modes using a graph-based state machine. */
public class DriveStateMachine extends SubsystemBase {

    /** Graph helper that orchestrates drive commands as state transitions. */
    private GraphCommand m_graphCommand = new GraphCommand();
    /** Dashboard binding used to visualize the current drive state. */
    private FlytDashboardV2 dashboard = new FlytDashboardV2("DriveStateMachine");
    private DriveSubsystem drive;
    private PoseEstimatorSubsystem pose;
    private XboxController driverController;

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
    GraphCommandNode manualNode;
    GraphCommandNode followPathNode;
    GraphCommandNode bargeRelativeNode;
    GraphCommandNode climbRelativeNode;
    GraphCommandNode processorRelativeNode;
    GraphCommandNode coralStationNode;
    GraphCommandNode reefRelativeNode;
    GraphCommandNode reefAlignNode;
    GraphCommandNode cancelledNode;

    // State variables.
    /** True when the driver wants to score on the right branch of the reef. */
    private boolean rightScore = false;

    /**
     * Constructor for the DriveStateMachine
     * @param m_drive
     * @param m_pose
     * @param m_driverController
     */
    public DriveStateMachine(DriveSubsystem m_drive, PoseEstimatorSubsystem m_pose, XboxController m_driverController) {
        drive = m_drive;
        pose = m_pose;
        driverController = m_driverController;

        // Initialize graph command.
        initializeGraphCommand();

        // Set as default command so it runs all the time.
        m_graphCommand.addRequirements(this);
        this.setDefaultCommand(m_graphCommand);
        m_graphCommand.setCurrentNode(cancelledNode);
    }

    /**
     * Builds the directed graph that maps each {@link DriveState} to the command that should run
     * while the state is active. Each call to {@link GraphCommandNode#AddNode} defines an allowed
     * transition between states.
     */
    private void initializeGraphCommand() {
        // Create all graph command nodes.
        manualNode = m_graphCommand.new GraphCommandNode(
            "Manual",
            new PrintCommand(""),
            new PrintCommand(""),
            DriveCommandFactory.createManualCommand(this, drive, pose, driverController));

        followPathNode = m_graphCommand.new GraphCommandNode(
            "FollowPath",
            DriveCommandFactory.createFollowPathCommand(this, drive, pose, driverController),
            new PrintCommand(""),
            new PrintCommand(""));

        bargeRelativeNode = m_graphCommand.new GraphCommandNode(
            "BargeRelative",
            new PrintCommand(""),
            new PrintCommand(""),
            DriveCommandFactory.createBargeRelativeCommand(this, drive, pose, driverController));

        climbRelativeNode = m_graphCommand.new GraphCommandNode(
            "ClimbRelative",
            new PrintCommand(""),
            new PrintCommand(""),
            DriveCommandFactory.createClimbRelativeCommand(this, drive, pose, driverController));

        processorRelativeNode = m_graphCommand.new GraphCommandNode(
            "ProcessorRelative",
            new PrintCommand(""),
            new PrintCommand(""),
            DriveCommandFactory.createProcessorRelativeCommand(this, drive, pose, driverController));


        coralStationNode = m_graphCommand.new GraphCommandNode(
            "CoralStation",
            new PrintCommand(""),
            new PrintCommand(""),
            DriveCommandFactory.createCoralStationCommand(this, drive, pose, driverController));

        reefRelativeNode = m_graphCommand.new GraphCommandNode(
            "ReefRelative",
            new PrintCommand(""),
            new PrintCommand(""),
            DriveCommandFactory.createReefRelativeCommand(this, drive, pose, driverController));

        reefAlignNode = m_graphCommand.new GraphCommandNode(
             "ReefAlign",
             new PrintCommand(""),
             new PrintCommand(""),
             DriveCommandFactory.createReefAlignCommand(this, drive, pose, driverController));

        cancelledNode = m_graphCommand.new GraphCommandNode(
            "Cancelled",
            new PrintCommand(""),
            new PrintCommand(""),
            DriveCommandFactory.createCancelledCommand(drive));

        // Graph Command setup.
        m_graphCommand.setGraphRootNode(cancelledNode); // Should not drive.

        // Define transitions between drive states.
        cancelledNode.AddNode(manualNode, 1.0);
        cancelledNode.AddNode(processorRelativeNode, 1.0);
        cancelledNode.AddNode(coralStationNode, 1.0);
        cancelledNode.AddNode(reefRelativeNode, 1.0);
        cancelledNode.AddNode(followPathNode, 1.0);
        cancelledNode.AddNode(bargeRelativeNode, 1.0);
        cancelledNode.AddNode(climbRelativeNode, 1.0);
        cancelledNode.AddNode(reefAlignNode, 1.0);
        manualNode.AddNode(cancelledNode, 1.0);
        manualNode.AddNode(processorRelativeNode, 1.0);
        manualNode.AddNode(coralStationNode, 1.0);
        manualNode.AddNode(reefRelativeNode, 1.0);
        manualNode.AddNode(followPathNode, 1.0);
        manualNode.AddNode(bargeRelativeNode, 1.0);
        manualNode.AddNode(climbRelativeNode, 1.0);
        manualNode.AddNode(reefAlignNode, 1.0);
        processorRelativeNode.AddNode(manualNode, 1.0);
        processorRelativeNode.AddNode(cancelledNode, 1.0);
        processorRelativeNode.AddNode(coralStationNode, 1.0);
        processorRelativeNode.AddNode(reefRelativeNode, 1.0);
        processorRelativeNode.AddNode(followPathNode, 1.0);
        processorRelativeNode.AddNode(bargeRelativeNode, 1.0);
        processorRelativeNode.AddNode(climbRelativeNode, 1.0);
        processorRelativeNode.AddNode(reefAlignNode, 1.0);
        coralStationNode.AddNode(manualNode, 1.0);
        coralStationNode.AddNode(cancelledNode, 1.0);
        coralStationNode.AddNode(processorRelativeNode, 1.0);
        coralStationNode.AddNode(reefRelativeNode, 1.0);
        coralStationNode.AddNode(followPathNode, 1.0);
        coralStationNode.AddNode(bargeRelativeNode, 1.0);
        coralStationNode.AddNode(climbRelativeNode, 1.0);
        coralStationNode.AddNode(reefAlignNode, 1.0);
        reefRelativeNode.AddNode(manualNode, 1.0);
        reefRelativeNode.AddNode(cancelledNode, 1.0);
        reefRelativeNode.AddNode(processorRelativeNode, 1.0);
        reefRelativeNode.AddNode(coralStationNode, 1.0);
        reefRelativeNode.AddNode(followPathNode, 1.0);
        reefRelativeNode.AddNode(bargeRelativeNode, 1.0);
        reefRelativeNode.AddNode(climbRelativeNode, 1.0);
        reefRelativeNode.AddNode(reefAlignNode, 1.0);
        followPathNode.AddNode(cancelledNode, 1.0);
        followPathNode.AddNode(manualNode, 1.0);
        followPathNode.AddNode(processorRelativeNode, 1.0);
        followPathNode.AddNode(coralStationNode, 1.0);
        followPathNode.AddNode(reefRelativeNode, 1.0);
        followPathNode.AddNode(bargeRelativeNode, 1.0);
        followPathNode.AddNode(climbRelativeNode, 1.0);
        followPathNode.AddNode(reefAlignNode, 1.0);
        bargeRelativeNode.AddNode(cancelledNode, 1.0);
        bargeRelativeNode.AddNode(manualNode, 1.0);
        bargeRelativeNode.AddNode(processorRelativeNode, 1.0);
        bargeRelativeNode.AddNode(coralStationNode, 1.0);
        bargeRelativeNode.AddNode(reefRelativeNode, 1.0);
        bargeRelativeNode.AddNode(followPathNode, 1.0);
        bargeRelativeNode.AddNode(climbRelativeNode, 1.0);
        bargeRelativeNode.AddNode(reefAlignNode, 1.0);
        climbRelativeNode.AddNode(cancelledNode, 1.0);
        climbRelativeNode.AddNode(manualNode, 1.0);
        climbRelativeNode.AddNode(processorRelativeNode, 1.0);
        climbRelativeNode.AddNode(coralStationNode, 1.0);
        climbRelativeNode.AddNode(reefRelativeNode, 1.0);
        climbRelativeNode.AddNode(followPathNode, 1.0);
        climbRelativeNode.AddNode(bargeRelativeNode, 1.0);
        climbRelativeNode.AddNode(reefAlignNode, 1.0);
        reefAlignNode.AddNode(cancelledNode, 1.0);
        reefAlignNode.AddNode(manualNode, 1.0);
        reefAlignNode.AddNode(processorRelativeNode, 1.0);
        reefAlignNode.AddNode(coralStationNode, 1.0);
        reefAlignNode.AddNode(reefRelativeNode, 1.0);
        reefAlignNode.AddNode(followPathNode, 1.0);
        reefAlignNode.AddNode(bargeRelativeNode, 1.0);
        reefAlignNode.AddNode(climbRelativeNode, 1.0);

    }

    // Branch selection helpers.

    /** Returns true when the driver selected the right reef branch. */
    public boolean getRightScore() {
        return rightScore;
    }

    /** Sets which reef branch the driver wants to score on. */
    public void setRightScore(boolean right) {
        // true means the driver wants to aim for the right branch, false targets the left.
        rightScore = right;
    }


    // State transition commands.

    /**
     * Requests a new drive state. The underlying {@link GraphCommand} handles whatever transitions
     * are necessary to reach the node safely.
     */
    public void setDriveCommand(DriveState m_driveState){
        switch(m_driveState){
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
     * @return {@code true} while the graph is still traversing toward the requested state.
     */
    public boolean isTransitioning() {
        return m_graphCommand.isTransitioning();
    }

    /** @return true when the pose estimator reports the drivetrain is at the requested pose. */
    public boolean atPosition(){
        return pose.atTargetPose();
    }
    // State getters.

    /** Returns the drive state associated with the current graph node. */
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


    // Periodic bookkeeping.
    @Override
    public void periodic() {
        // Update dashboard with the human-friendly state name.
        dashboard.putString("Current State", getCurrentState().toString());
        dashboard.putBoolean("At State", !isTransitioning());
        dashboard.putString("Branch", getRightScore() ? "Right" : "Left");
        dashboard.putBoolean("At Drive Position", atPosition());
        
    }
}