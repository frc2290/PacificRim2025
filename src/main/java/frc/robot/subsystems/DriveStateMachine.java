package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.GraphCommand;
import frc.robot.commands.GraphCommand.GraphCommandNode;
import frc.robot.commands.DriveCommands.CoralStationDrive;
import frc.robot.commands.DriveCommands.ManualDrive;
import frc.robot.commands.DriveCommands.ProcessorRelativeDrive;
import frc.robot.commands.DriveCommands.ReefRelativeDrive;
import frc.utils.PoseEstimatorSubsystem;

public class DriveStateMachine extends SubsystemBase {

    private GraphCommand m_graphCommand = new GraphCommand();
    private DriveSubsystem drive;
    private PoseEstimatorSubsystem pose;
    private XboxController driverController;

    /**
     * DriveTrain states - drive state machine
     */
    public enum DriveState {
        MANUAL,            // Field oriented freeroam
        FOLLOW_PATH,       // Auto path following
        BARGE_RELATIVE,    // Faces Barge
        CLIMB_RELATIVE,    // Faces Climb
        PROCESSOR_RELATIVE, // Faces Processor
        CORAL_STATION,     // Faces Intake based on half field
        REEF_RELATIVE,     // Faces Reef based on robot position and angles as drives around
        REEF_ALIGN,        // Locked to right or left reef, holding position
        CANCELLED          // Drive system cancelled
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
    private boolean rotLock = true;
    private boolean rightScore = false;
    private boolean isDisabled = false;

    /**
     * Create a new DriveStateMachine
     */
    public DriveStateMachine(DriveSubsystem m_drive, PoseEstimatorSubsystem m_pose, XboxController m_driverController) {
        drive = m_drive;
        pose = m_pose;
        driverController = m_driverController;

        // Initialize graph command
        initializeGraphCommand();

        // Set as default command so it runs all the time
        m_graphCommand.addRequirements(this);
        this.setDefaultCommand(m_graphCommand);
        m_graphCommand.setCurrentNode(manualNode);
        m_graphCommand.initialize();
    }

    /**
     * Initialize the graph command with all nodes and connections
     */
    private void initializeGraphCommand() {
        // Create all graph command nodes
        manualNode = m_graphCommand.new GraphCommandNode(
            "Manual", 
            new ManualDrive(this, drive, pose, driverController),
            new PrintCommand("Arrived at Manual state"),
            new PrintCommand("Arrived at Manual state"));
        
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

        processorRelativeNode = m_graphCommand.new GraphCommandNode(
            "ProcessorRelative",
            new ProcessorRelativeDrive(this, drive, pose, driverController),
            new PrintCommand("Arrived at Teleop state"),
            new PrintCommand("Arrived at ProcessorRelative state"));

        coralStationNode = m_graphCommand.new GraphCommandNode(
            "CoralStation",
            new CoralStationDrive(this, drive, pose, driverController),
            new PrintCommand("Arrived at Teleop state"),
            new PrintCommand("Arrived at CoralStation state"));

        reefRelativeNode = m_graphCommand.new GraphCommandNode(
            "ReefRelative",
            new ReefRelativeDrive(this, drive, pose, driverController),
            new PrintCommand("Arrived at Teleop state"),
            new PrintCommand("Arrived at ReefRelative state"));

        // reefAlignNode = m_graphCommand.new GraphCommandNode(
        //     "ReefAlign",
        //     new ReefAlignDrive(this, drive, pose, driverController),
        //     new InstantCommand(() -> drive.stop()),
        //     new PrintCommand("Arrived at ReefAlign state"));

        // cancelledNode = m_graphCommand.new GraphCommandNode(
        //     "Cancelled",
        //     new InstantCommand(() -> drive.stop()),
        //     new InstantCommand(() -> drive.stop()),
        //     new PrintCommand("Arrived at Cancelled state"));

        // Graph Command setup
        m_graphCommand.setGraphRootNode(manualNode);
        
        // Define transitions between drive states
        //teleopNode.AddNode(followPathNode, 1.0);
        //teleopNode.AddNode(bargeRelativeNode, 1.0);
        //teleopNode.AddNode(climbRelativeNode, 1.0);
        manualNode.AddNode(processorRelativeNode, 1.0);
        manualNode.AddNode(coralStationNode, 1.0);
        manualNode.AddNode(reefRelativeNode, 1.0);
        //teleopNode.AddNode(reefAlignNode, 1.0);
        //teleopNode.AddNode(cancelledNode, 1.0);
        
        //followPathNode.AddNode(teleopNode, 1.0);
        //followPathNode.AddNode(cancelledNode, 1.0);
        
        //bargeRelativeNode.AddNode(teleopNode, 1.0);
        //bargeRelativeNode.AddNode(cancelledNode, 1.0);
        
        //climbRelativeNode.AddNode(teleopNode, 1.0);
        //climbRelativeNode.AddNode(cancelledNode, 1.0);
        
        processorRelativeNode.AddNode(manualNode, 1.0);
        //processorRelativeNode.AddNode(cancelledNode, 1.0);
        
        coralStationNode.AddNode(manualNode, 1.0);
        //coralStationNode.AddNode(cancelledNode, 1.0);
        
        reefRelativeNode.AddNode(manualNode, 1.0);
        //reefRelativeNode.AddNode(reefAlignNode, 1.0);
        //reefRelativeNode.AddNode(cancelledNode, 1.0);
        
        //reefAlignNode.AddNode(reefRelativeNode, 1.0);
        //reefAlignNode.AddNode(teleopNode, 1.0);
        //reefAlignNode.AddNode(cancelledNode, 1.0);
        
        //cancelledNode.AddNode(teleopNode, 1.0);
    }

    /** ----- Branch Selection ----- */
    public boolean getRightScore() {
        return rightScore;
    }

    public void setRightScore(boolean right) {
        rightScore = right;
    }

    public Command setRightScoreCommand(boolean right) {
        return new InstantCommand(() -> setRightScore(right));
    }

    /** ----- Rotation Lock ----- */
    public boolean getRotationLock() {
        return rotLock;
    }

    public void setRotationLock(boolean lock) {
        rotLock = lock;
    }

    public void setDisabled(boolean disabled) {
        isDisabled = disabled;
    }

    public Command toggleRotationLock() {
        return new InstantCommand(() -> rotLock = !rotLock);
    }
    

    /** ----- State Transition Commands ----- */
    public Command setManualCommand() {
        return new InstantCommand(() -> m_graphCommand.setTargetNode(manualNode));
    }

    public Command setFollowPathCommand() {
        return new InstantCommand(() -> m_graphCommand.setTargetNode(followPathNode));
    }

    public Command setBargeRelativeCommand() {
        return new InstantCommand(() -> m_graphCommand.setTargetNode(bargeRelativeNode));
    }

    public Command setClimbRelativeCommand() {
        return new InstantCommand(() -> m_graphCommand.setTargetNode(climbRelativeNode));
    }

    public Command setProcessorRelativeCommand() {
        return new InstantCommand(() -> m_graphCommand.setTargetNode(processorRelativeNode));
    }

    public Command setCoralStationCommand() {
        return new InstantCommand(() -> m_graphCommand.setTargetNode(coralStationNode));
    }

    public Command setReefRelativeCommand() {
        return new InstantCommand(() -> m_graphCommand.setTargetNode(reefRelativeNode));
    }

    public Command setReefAlignCommand() {
        return new InstantCommand(() -> m_graphCommand.setTargetNode(reefAlignNode));
    }

    public Command setCancelledCommand() {
        return new InstantCommand(() -> m_graphCommand.setTargetNode(cancelledNode));
    }


    /** ----- State Getters ----- */
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

    public boolean isTransitioning() {
        return m_graphCommand.isTransitioning();
    }

    

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}