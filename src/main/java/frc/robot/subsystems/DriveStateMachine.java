package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.GraphCommand;
import frc.robot.commands.GraphCommand.GraphCommandNode;
import frc.robot.commands.DriveCommands.CancelledDrive;
import frc.robot.commands.DriveCommands.CoralStationDrive;
import frc.robot.commands.DriveCommands.ManualDrive;
import frc.robot.commands.DriveCommands.ProcessorRelativeDrive;
import frc.robot.commands.DriveCommands.ReefRelativeDrive;
import frc.utils.FlytDashboardV2;
import frc.utils.PoseEstimatorSubsystem;

public class DriveStateMachine extends SubsystemBase {

    private GraphCommand m_graphCommand = new GraphCommand();
    private FlytDashboardV2 dashboard = new FlytDashboardV2("DriveStateMachine");
    private DriveSubsystem drive;
    private PoseEstimatorSubsystem pose;
    private XboxController driverController;
    private ElevatorSubsystem m_elevator;
    private DifferentialSubsystem m_mManipulatorSubsystem;

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
    public DriveStateMachine(DriveSubsystem m_drive, PoseEstimatorSubsystem m_pose, XboxController m_driverController, ElevatorSubsystem m_elevatorManipulator, DifferentialSubsystem m_ManipulatorSubsystem) {
        drive = m_drive;
        pose = m_pose;
        driverController = m_driverController;
        m_elevator = m_elevatorManipulator;
        m_mManipulatorSubsystem = m_ManipulatorSubsystem;

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

        processorRelativeNode = m_graphCommand.new GraphCommandNode(
            "ProcessorRelative",
            new PrintCommand(""),
            new PrintCommand(""),
            new ProcessorRelativeDrive(this, drive, pose, driverController));
        

        coralStationNode = m_graphCommand.new GraphCommandNode(
            "CoralStation",
            new PrintCommand(""),
            new PrintCommand(""),
            new CoralStationDrive(this, drive, pose, driverController));

        reefRelativeNode = m_graphCommand.new GraphCommandNode(
            "ReefRelative",
            new PrintCommand(""),
            new PrintCommand(""),
            new ReefRelativeDrive(this, drive, pose, driverController));

        // reefAlignNode = m_graphCommand.new GraphCommandNode(
        //     "ReefAlign",
        //     new ReefAlignDrive(this, drive, pose, driverController),
        //     new InstantCommand(() -> drive.stop()),
        //     new PrintCommand("Arrived at ReefAlign state"));

        cancelledNode = m_graphCommand.new GraphCommandNode(
            "Cancelled",
            new PrintCommand(""),
            new PrintCommand(""),
            new CancelledDrive(drive));

        // Graph Command setup
        m_graphCommand.setGraphRootNode(cancelledNode); //shouldn't drive
        
        // Define transitions between drive states
        cancelledNode.AddNode(manualNode, 1.0);
        cancelledNode.AddNode(processorRelativeNode, 1.0);
        cancelledNode.AddNode(coralStationNode, 1.0);
        cancelledNode.AddNode(reefRelativeNode, 1.0);
        //cancelledNode.AddNode(followPathNode, 1.0);
        //cancelledNode.AddNode(bargeRelativeNode, 1.0);
        //cancelledNode.AddNode(climbRelativeNode, 1.0);
        //cancelledNode.AddNode(reefAlignNode, 1.0);
        manualNode.AddNode(cancelledNode, 1.0);
        manualNode.AddNode(processorRelativeNode, 1.0);
        manualNode.AddNode(coralStationNode, 1.0);
        manualNode.AddNode(reefRelativeNode, 1.0);
        //manualNode.AddNode(followPathNode, 1.0);
        //manualNode.AddNode(bargeRelativeNode, 1.0);
        //manualNode.AddNode(climbRelativeNode, 1.0);
        //manualNode.AddNode(reefAlignNode, 1.0);
        processorRelativeNode.AddNode(manualNode, 1.0);
        processorRelativeNode.AddNode(cancelledNode, 1.0);
        processorRelativeNode.AddNode(coralStationNode, 1.0);
        processorRelativeNode.AddNode(reefRelativeNode, 1.0);
        //processorRelativeNode.AddNode(followPathNode, 1.0);
        //processorRelativeNode.AddNode(bargeRelativeNode, 1.0);
        //processorRelativeNode.AddNode(climbRelativeNode, 1.0);
        //processorRelativeNode.AddNode(reefAlignNode, 1.0);
        coralStationNode.AddNode(manualNode, 1.0);
        coralStationNode.AddNode(cancelledNode, 1.0);
        coralStationNode.AddNode(processorRelativeNode, 1.0);
        coralStationNode.AddNode(reefRelativeNode, 1.0);
        //coralStationNode.AddNode(followPathNode, 1.0);
        //coralStationNode.AddNode(bargeRelativeNode, 1.0);
        //coralStationNode.AddNode(climbRelativeNode, 1.0);
        //coralStationNode.AddNode(reefAlignNode, 1.0);
        reefRelativeNode.AddNode(manualNode, 1.0);
        reefRelativeNode.AddNode(cancelledNode, 1.0);
        reefRelativeNode.AddNode(processorRelativeNode, 1.0);
        reefRelativeNode.AddNode(coralStationNode, 1.0);
        //reefRelativeNode.AddNode(followPathNode, 1.0);
        //reefRelativeNode.AddNode(bargeRelativeNode, 1.0);
        //reefRelativeNode.AddNode(climbRelativeNode, 1.0);
        //reefRelativeNode.AddNode(reefAlignNode, 1.0); 

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
    public Command setDriveCommand(DriveState m_driveState){
        switch(m_driveState){
            case MANUAL:
                return new InstantCommand(() -> m_graphCommand.setTargetNode(manualNode));
            case FOLLOW_PATH:
                return new InstantCommand(() -> m_graphCommand.setTargetNode(followPathNode));
            case BARGE_RELATIVE:
                return new InstantCommand(() -> m_graphCommand.setTargetNode(bargeRelativeNode));
            case CLIMB_RELATIVE:
                return new InstantCommand(() -> m_graphCommand.setTargetNode(climbRelativeNode));
            case PROCESSOR_RELATIVE:
                return new InstantCommand(() -> m_graphCommand.setTargetNode(processorRelativeNode));
            case CORAL_STATION:
                return new InstantCommand(() -> m_graphCommand.setTargetNode(coralStationNode));
            case REEF_RELATIVE:
                return new InstantCommand(() -> m_graphCommand.setTargetNode(reefRelativeNode));
            case REEF_ALIGN:
                return new InstantCommand(() -> m_graphCommand.setTargetNode(reefAlignNode));
            case CANCELLED:
                return new InstantCommand(() -> m_graphCommand.setTargetNode(cancelledNode));
            default:
                return new InstantCommand(() -> m_graphCommand.setTargetNode(manualNode));
        }
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
        // Handle automatic transitions
        handleAutomaticTransitions();
    
        //Update dashboard
        dashboard.putString("Current State", getCurrentState().toString());
        dashboard.putBoolean("Transitioning", isTransitioning());
        dashboard.putString("Branch", getRightScore() ? "Right" : "Left");
        
    }

        /**
     * Handle automatic state transitions
     */
    private void handleAutomaticTransitions() {

        //If robot is enabled and drive was cancelled, return to manual drive state
        //if(getCurrentState() == DriveState.CANCELLED && !isDisabled) setDriveCommand(DriveState.MANUAL).schedule();)

        //Safe Disable robot if disabled
        //if(isDisabled && getCurrentState() != DriveState.CANCELLED) setDriveCommand(DriveState.CANCELLED).schedule();

       //Get state of elevator from coardinator and see if we are going for coral intake, if we have note, algae, climb etc.
        


        // // Timeout protection for intake
        // if (getCurrentState() == ElevatorManipulatorState.IntakeCoral && 
        //     Timer.getFPGATimestamp() - stateEntryTime > 5.0) {
        //     DataLogManager.log("ElevManiSM: Intake timeout - returning to safe");
        //     requestState(ElevatorManipulatorState.SafeCoralTravel);
        // }
    }
}