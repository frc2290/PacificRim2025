package frc.robot.subsystems;

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.commands.GraphCommand;
import frc.robot.subsystems.DifferentialSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;

/**
 * Elevator + Manipulator State Machine using GraphCommand
 * Handles all coordinated movements between elevator, differential arm, and manipulator
 */
public class ElevatorManipulatorStateMachine extends SubsystemBase {
    private final GraphCommand graphCommand;
    private final DifferentialSubsystem diff;
    private final ElevatorSubsystem elevator;
    private final ManipulatorSubsystem manipulator;
    
    // State tracking
    private String currentState = "SafeCoralTravel";
    private String goalState = "SafeCoralTravel";
    private boolean isTransitioning = false;
    private double stateEntryTime = 0;
    
    // Game piece tracking
    private boolean hasCoral = false;
    private boolean hasAlgae = false;
    
    // Node references for easy access
    private GraphCommand.GraphCommandNode safeCoralTravelNode;
    private GraphCommand.GraphCommandNode intakeCoralNode;
    private GraphCommand.GraphCommandNode l4ScoreNode;
    private GraphCommand.GraphCommandNode l3ScoreNode;
    private GraphCommand.GraphCommandNode l2ScoreNode;
    private GraphCommand.GraphCommandNode l1ScoreNode;
    private GraphCommand.GraphCommandNode algaeL3Node;
    private GraphCommand.GraphCommandNode algaeL2Node;
    private GraphCommand.GraphCommandNode processorNode;
    private GraphCommand.GraphCommandNode manualNode;
    private GraphCommand.GraphCommandNode climbPrepNode;

    public ElevatorManipulatorStateMachine(DifferentialSubsystem diff, 
                                         ElevatorSubsystem elevator, 
                                         ManipulatorSubsystem manipulator) {
        this.diff = diff;
        this.elevator = elevator;
        this.manipulator = manipulator;
        
        this.graphCommand = buildStateMachine();
        this.graphCommand.schedule();
        
        // Initialize game piece tracking
        updateGamePieceStatus();
    }
    
    /**
     * Build the complete state machine graph
     */
    private GraphCommand buildStateMachine() {
        GraphCommand graph = new GraphCommand();
        
        // Create all state nodes
        safeCoralTravelNode = createNode("SafeCoralTravel", 
            new SafeTravelCommand(),
            new SafeTravelCommand(),
            new SafeTravelCompleteCommand());
        
        intakeCoralNode = createNode("IntakeCoral",
            new IntakeCoralCommand(),
            new PrepareIntakeCommand(),
            new IntakeCompleteCommand());
        
        l4ScoreNode = createNode("L4Score",
            new L4ScoreCommand(),
            new PrepareL4Command(),
            new ScoreCompleteCommand());
        
        l3ScoreNode = createNode("L3Score",
            new L3ScoreCommand(),
            new PrepareL3Command(),
            new ScoreCompleteCommand());
        
        l2ScoreNode = createNode("L2Score",
            new L2ScoreCommand(),
            new PrepareL2Command(),
            new ScoreCompleteCommand());
        
        l1ScoreNode = createNode("L1Score",
            new L1ScoreCommand(),
            new PrepareL1Command(),
            new ScoreCompleteCommand());
        
        algaeL3Node = createNode("AlgaeL3",
            new AlgaeL3Command(),
            new PrepareAlgaeL3Command(),
            new AlgaeScoreCompleteCommand());
        
        algaeL2Node = createNode("AlgaeL2",
            new AlgaeL2Command(),
            new PrepareAlgaeL2Command(),
            new AlgaeScoreCompleteCommand());
        
        processorNode = createNode("Processor",
            new ProcessorCommand(),
            new PrepareProcessorCommand(),
            new ProcessorCompleteCommand());
        
        manualNode = createNode("Manual",
            new ManualControlCommand(),
            new ManualControlCommand(),
            Commands.none());
        
        climbPrepNode = createNode("ClimbPrep",
            new ClimbPrepCommand(),
            new ClimbPrepCommand(),
            new ClimbReadyCommand());
        
        // Connect nodes with appropriate costs (time estimates in seconds)
        // Safe travel connections
        safeCoralTravelNode.AddNode(intakeCoralNode, 2.0);
        safeCoralTravelNode.AddNode(l4ScoreNode, 3.0);
        safeCoralTravelNode.AddNode(l3ScoreNode, 2.5);
        safeCoralTravelNode.AddNode(l2ScoreNode, 2.0);
        safeCoralTravelNode.AddNode(l1ScoreNode, 1.5);
        safeCoralTravelNode.AddNode(algaeL3Node, 2.5);
        safeCoralTravelNode.AddNode(algaeL2Node, 2.0);
        safeCoralTravelNode.AddNode(processorNode, 1.5);
        safeCoralTravelNode.AddNode(climbPrepNode, 3.0);
        
        // Return paths from scoring positions
        l4ScoreNode.AddNode(safeCoralTravelNode, 2.5);
        l3ScoreNode.AddNode(safeCoralTravelNode, 2.0);
        l2ScoreNode.AddNode(safeCoralTravelNode, 1.5);
        l1ScoreNode.AddNode(safeCoralTravelNode, 1.0);
        algaeL3Node.AddNode(safeCoralTravelNode, 2.0);
        algaeL2Node.AddNode(safeCoralTravelNode, 1.5);
        processorNode.AddNode(safeCoralTravelNode, 1.0);
        climbPrepNode.AddNode(safeCoralTravelNode, 2.5);
        
        // Manual mode can be reached from anywhere (high cost to discourage accidental use)
        safeCoralTravelNode.AddNode(manualNode, 10.0);
        intakeCoralNode.AddNode(manualNode, 10.0);
        l4ScoreNode.AddNode(manualNode, 10.0);
        manualNode.AddNode(safeCoralTravelNode, 1.0);
        
        graph.setGraphRootNode(safeCoralTravelNode);
        return graph;
    }
    
    /**
     * Request a state transition
     */
    public void requestState(String stateName) {
        GraphCommand.GraphCommandNode targetNode = findNodeByName(stateName);
        if (targetNode != null && !goalState.equals(stateName)) {
            DataLogManager.log("ElevManiSM: Requesting state " + stateName);
            goalState = stateName;
            graphCommand.setTargetNode(targetNode);
        }
    }
    
    /**
     * Emergency return to safe travel
     */
    public void emergencyReturnToSafe() {
        DataLogManager.log("ElevManiSM: EMERGENCY - Returning to safe travel");
        requestState("SafeCoralTravel");
    }
    
    /**
     * Cancel current operation and hold position
     */
    public void cancelAndHold() {
        DataLogManager.log("ElevManiSM: Canceling and holding position");
        if (graphCommand.isTransitioning()) {
            graphCommand.cancel();
        }
        elevator.holdPosition();
        diff.holdPosition();
        manipulator.stop();
    }
    
    /**
     * Get current state name
     */
    public String getCurrentState() {
        return currentState;
    }
    
    /**
     * Get goal state name
     */
    public String getGoalState() {
        return goalState;
    }
    
    /**
     * Check if currently transitioning
     */
    public boolean isTransitioning() {
        return isTransitioning;
    }
    
    /**
     * Check if has coral
     */
    public boolean hasCoral() {
        return hasCoral;
    }
    
    /**
     * Check if has algae
     */
    public boolean hasAlgae() {
        return hasAlgae;
    }
    
    /**
     * Check if at safe travel position
     */
    public boolean isInSafePosition() {
        return currentState.equals("SafeCoralTravel") && 
               elevator.atPosition() && 
               diff.atExtensionSetpoint() && 
               diff.atRotationSetpoint();
    }
    
    /**
     * Check if ready to score
     */
    public boolean isReadyToScore() {
        return (currentState.equals("L4Score") || currentState.equals("L3Score") || 
                currentState.equals("L2Score") || currentState.equals("L1Score")) &&
               elevator.atPosition() && 
               diff.atExtensionSetpoint() && 
               diff.atRotationSetpoint();
    }
    
    /**
     * Update game piece status from sensors
     */
    private void updateGamePieceStatus() {
        hasCoral = manipulator.hasCoral();
        hasAlgae = manipulator.hasAlgae();
    }
    
    /**
     * Find node by name
     */
    private GraphCommand.GraphCommandNode findNodeByName(String name) {
        // This would be implemented with a map of all nodes
        // For now, simple switch case
        switch (name) {
            case "SafeCoralTravel": return safeCoralTravelNode;
            case "IntakeCoral": return intakeCoralNode;
            case "L4Score": return l4ScoreNode;
            case "L3Score": return l3ScoreNode;
            case "L2Score": return l2ScoreNode;
            case "L1Score": return l1ScoreNode;
            case "AlgaeL3": return algaeL3Node;
            case "AlgaeL2": return algaeL2Node;
            case "Processor": return processorNode;
            case "Manual": return manualNode;
            case "ClimbPrep": return climbPrepNode;
            default:
                DataLogManager.log("ElevManiSM: Unknown state " + name);
                return null;
        }
    }
    
    @Override
    public void periodic() {
        if (!DriverStation.isEnabled()) return;
        
        updateGamePieceStatus();
        updateStateTracking();
        handleAutomaticTransitions();
        enforceSafetyLimits();
    }
    
    /**
     * Update current state tracking
     */
    private void updateStateTracking() {
        String previousState = currentState;
        currentState = graphCommand.getCurrentNode().getNodeName();
        isTransitioning = graphCommand.isTransitioning();
        
        if (!previousState.equals(currentState)) {
            stateEntryTime = Timer.getFPGATimestamp();
            DataLogManager.log("ElevManiSM: Entered state " + currentState);
        }
    }
    
    /**
     * Handle automatic state transitions
     */
    private void handleAutomaticTransitions() {
        // Automatically go to intake if we don't have coral and are in safe travel
        if (currentState.equals("SafeCoralTravel") && !hasCoral && !isTransitioning) {
            requestState("IntakeCoral");
        }
        
        // Return to safe travel after scoring
        if ((currentState.equals("L4Score") || currentState.equals("L3Score") || 
             currentState.equals("L2Score") || currentState.equals("L1Score")) && 
            !hasCoral && !isTransitioning) {
            // Wait a moment to ensure score is complete
            if (Timer.getFPGATimestamp() - stateEntryTime > 0.5) {
                requestState("SafeCoralTravel");
            }
        }
        
        // Timeout protection for intake
        if (currentState.equals("IntakeCoral") && 
            Timer.getFPGATimestamp() - stateEntryTime > 5.0) {
            DataLogManager.log("ElevManiSM: Intake timeout - returning to safe");
            requestState("SafeCoralTravel");
        }
    }
    
    /**
     * Enforce safety limits and constraints
     */
    private void enforceSafetyLimits() {
        // Prevent elevator movement when differential arm is extended beyond safe limits
        if (diff.getExtensionPosition() > DiffArmConstants.SAFE_EXTENSION_FOR_ELEVATOR_MOVEMENT && 
            elevator.isMoving()) {
            DataLogManager.log("ElevManiSM: SAFETY - Stopping elevator due to arm extension");
            elevator.stop();
        }
        
        // Prevent arm extension when elevator is too high
        if (elevator.getPosition() > ElevatorConstants.SAFE_HEIGHT_FOR_ARM_EXTENSION && 
            diff.isExtending()) {
            DataLogManager.log("ElevManiSM: SAFETY - Stopping arm extension due to elevator height");
            diff.stopExtension();
        }
    }
    
    // ===== COMMAND IMPLEMENTATIONS =====
    
    private class SafeTravelCommand extends Command {
        public SafeTravelCommand() {
            addRequirements(elevator, diff, manipulator);
        }
        
        @Override
        public void initialize() {
            elevator.setTargetPosition(ElevatorConstants.SAFE_TRAVEL_HEIGHT);
            diff.setTargets(DiffArmConstants.SAFE_EXTENSION, DiffArmConstants.SAFE_ROTATION);
            manipulator.setIntakeSpeed(0);
        }
        
        @Override
        public boolean isFinished() {
            return elevator.atPosition() && 
                   diff.atExtensionSetpoint() && 
                   diff.atRotationSetpoint();
        }
    }
    
    private class IntakeCoralCommand extends Command {
        private final Timer intakeTimer = new Timer();
        
        public IntakeCoralCommand() {
            addRequirements(elevator, diff, manipulator);
        }
        
        @Override
        public void initialize() {
            elevator.setTargetPosition(ElevatorConstants.INTAKE_HEIGHT);
            diff.setTargets(DiffArmConstants.INTAKE_EXTENSION, DiffArmConstants.INTAKE_ROTATION);
            manipulator.setIntakeSpeed(ManipulatorConstants.INTAKE_SPEED);
            intakeTimer.restart();
        }
        
        @Override
        public boolean isFinished() {
            return hasCoral || intakeTimer.hasElapsed(4.0);
        }
        
        @Override
        public void end(boolean interrupted) {
            manipulator.setIntakeSpeed(0);
        }
    }
    
    private class L4ScoreCommand extends Command {
        public L4ScoreCommand() {
            addRequirements(elevator, diff, manipulator);
        }
        
        @Override
        public void initialize() {
            elevator.setTargetPosition(ElevatorConstants.L4_HEIGHT);
            diff.setTargets(DiffArmConstants.L4_EXTENSION, DiffArmConstants.L4_ROTATION);
        }
        
        @Override
        public boolean isFinished() {
            return elevator.atPosition() && 
                   diff.atExtensionSetpoint() && 
                   diff.atRotationSetpoint();
        }
    }
    
    private class ScoreCompleteCommand extends Command {
        private final Timer scoreTimer = new Timer();
        
        public ScoreCompleteCommand() {
            addRequirements(manipulator);
        }
        
        @Override
        public void initialize() {
            scoreTimer.restart();
            manipulator.setIntakeSpeed(ManipulatorConstants.SCORE_SPEED);
        }
        
        @Override
        public boolean isFinished() {
            return scoreTimer.hasElapsed(1.0) || !hasCoral;
        }
        
        @Override
        public void end(boolean interrupted) {
            manipulator.setIntakeSpeed(0);
        }
    }
    
    // Additional command implementations would go here...
    // L3ScoreCommand, L2ScoreCommand, L1ScoreCommand, AlgaeL3Command, etc.
    
    private class ManualControlCommand extends Command {
        public ManualControlCommand() {
            addRequirements(elevator, diff, manipulator);
        }
        
        @Override
        public void initialize() {
            // In manual mode, we don't set targets - manual control handles this
            DataLogManager.log("ElevManiSM: Manual control enabled");
        }
        
        @Override
        public boolean isFinished() {
            return false; // Manual mode stays active until state change
        }
        
        @Override
        public void end(boolean interrupted) {
            DataLogManager.log("ElevManiSM: Manual control disabled");
        }
    }
    
    // ===== PUBLIC API FOR EXTERNAL CONTROL =====
    
    public void setManualElevatorPower(double power) {
        if (currentState.equals("Manual")) {
            elevator.setPower(power);
        }
    }
    
    public void setManualArmExtension(double power) {
        if (currentState.equals("Manual")) {
            diff.setExtensionPower(power);
        }
    }
    
    public void setManualArmRotation(double power) {
        if (currentState.equals("Manual")) {
            diff.setRotationPower(power);
        }
    }
    
    public void setManualIntakeSpeed(double speed) {
        if (currentState.equals("Manual")) {
            manipulator.setIntakeSpeed(speed);
        }
    }
    
    /**
     * Get diagnostic information
     */
    public String getDiagnostics() {
        return String.format("State=%s, Goal=%s, Transitioning=%s, HasCoral=%s, HasAlgae=%s",
            currentState, goalState, isTransitioning, hasCoral, hasAlgae);
    }
}