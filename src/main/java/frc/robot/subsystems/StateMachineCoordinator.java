package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * Coordinates interactions between DriveStateMachine and ElevatorManipulatorStateMachine
 * Handles safety interlocks, automated sequences, and state dependencies
 */
public class StateMachineCoordinator extends SubsystemBase {
    private final ElevatorManipulatorStateMachine elevManiSM;
    private final DriveStateMachine driveSM;
    
    // State tracking
    private boolean isAutoMode = false;
    private boolean safetyOverride = false;
    
    // Timers for delayed actions
    private double lastStateChangeTime = 0;
    
    public StateMachineCoordinator(ElevatorManipulatorStateMachine elevManiSM,
                                 DriveStateMachine driveSM) {
        this.elevManiSM = elevManiSM;
        this.driveSM = driveSM;
    }
    
    @Override
    public void periodic() {
        if (!DriverStation.isEnabled()) return;
        
        coordinateStateTransitions();
        enforceSafetyInterlocks();
        handleAutomatedBehaviors();
        updateLEDsBasedOnState();
    }
    
    /**
     * Main coordination logic between state machines
     */
    private void coordinateStateTransitions() {
        String elevState = elevManiSM.getCurrentState();
        DriveStateMachine.DriveState driveState = driveSM.getCurrentState();
        
        // EXAMPLE 1: When intaking coral, automatically face coral station
        if (elevState.equals("IntakeCoral") && 
            !driveState.equals(DriveStateMachine.DriveState.CoralStation)) {
            DataLogManager.log("Coordinator: Intaking -> Facing Coral Station");
            driveSM.requestState(DriveStateMachine.DriveState.CoralStation);
        }
        
        // EXAMPLE 2: When scoring at L4, automatically align to reef
        if ((elevState.equals("L4Score") || elevState.equals("L3Score")) && 
            !driveState.equals(DriveStateMachine.DriveState.ReefAlign)) {
            DataLogManager.log("Coordinator: Scoring -> Aligning to Reef");
            driveSM.requestState(DriveStateMachine.DriveState.ReefAlign);
        }
        
        // EXAMPLE 3: When in safe travel, return to teleop drive
        if (elevState.equals("SafeCoralTravel") && 
            !driveState.equals(DriveStateMachine.DriveState.Teleop) &&
            !isAutoMode) {
            DataLogManager.log("Coordinator: Safe Travel -> Teleop Drive");
            driveSM.requestState(DriveStateMachine.DriveState.Teleop);
        }
        
        // EXAMPLE 4: Prevent driving while elevator is in unsafe positions
        if (elevatorIsUnsafe() && !safetyOverride) {
            if (!driveState.equals(DriveStateMachine.DriveState.Cancelled)) {
                DataLogManager.log("Coordinator: Unsafe elevator -> Cancel drive");
                driveSM.requestState(DriveStateMachine.DriveState.Cancelled);
            }
        }
    }
    
    /**
     * Safety interlocks to prevent damage
     */
    private void enforceSafetyInterlocks() {
        // EXAMPLE 1: Prevent elevator movement while driving fast
        if (isDrivingAggressively() && !elevManiSM.getCurrentState().equals("SafeCoralTravel")) {
            DataLogManager.log("Coordinator: Aggressive driving -> Force safe travel");
            elevManiSM.requestState("SafeCoralTravel");
        }
        
        // EXAMPLE 2: Prevent scoring without game piece
        if (isScoringState() && !elevManiSM.hasCoral()) {
            DataLogManager.log("Coordinator: No coral -> Cancel scoring");
            elevManiSM.requestState("SafeCoralTravel");
        }
        
        // EXAMPLE 3: Safe elevator height before extending arm
        if (elevManiSM.getCurrentState().equals("IntakeCoral") && 
            !elevatorIsAtSafeHeightForIntake()) {
            DataLogManager.log("Coordinator: Elevator too high for intake");
            // You could add a pre-intake safe state here
        }
    }
    
    /**
     * Automated behaviors and sequences
     */
    private void handleAutomatedBehaviors() {
        // EXAMPLE 1: Automatic intake sequence
        if (elevManiSM.getCurrentState().equals("IntakeCoral") && 
            elevManiSM.hasCoral()) {
            // Wait a moment to ensure coral is secured
            if (lastStateChangeTime == 0) {
                lastStateChangeTime = Timer.getFPGATimestamp();
            }
            
            if (Timer.getFPGATimestamp() - lastStateChangeTime > 0.5) {
                DataLogManager.log("Coordinator: Coral acquired -> Safe travel");
                elevManiSM.requestState("SafeCoralTravel");
                lastStateChangeTime = 0;
            }
        }
        
        // EXAMPLE 2: Auto-align after scoring
        if (wasJustScoring() && driveSM.getCurrentState().equals(DriveStateMachine.DriveState.ReefAlign)) {
            // Consider switching to reef relative after successful score
            // This would be based on driver preference or auto routines
        }
    }
    
    /**
     * Update LEDs based on combined system state
     */
    private void updateLEDsBasedOnState() {
        // EXAMPLE: Different LED patterns based on what the robot is doing
        String elevState = elevManiSM.getCurrentState();
        
        if (elevState.equals("IntakeCoral")) {
            // Set LEDs to blinking blue for intake
            // ledUtility.setPattern(LEDPattern.BLINKING_BLUE);
        } 
        else if (elevState.equals("L4Score") || elevState.equals("L3Score")) {
            // Set LEDs to solid green for ready to score
            // ledUtility.setPattern(LEDPattern.SOLID_GREEN);
        }
        else if (elevatorIsUnsafe()) {
            // Set LEDs to red flashing for unsafe condition
            // ledUtility.setPattern(LEDPattern.FLASHING_RED);
        }
        else {
            // Default LED pattern
            // ledUtility.setPattern(LEDPattern.DEFAULT);
        }
    }
    
    // ===== HELPER METHODS =====
    
    private boolean elevatorIsUnsafe() {
        // Check if elevator is in a position that could be dangerous while moving
        // return elevManiSM.getElevatorHeight() > Constants.Safety.MAX_SAFE_HEIGHT_FOR_MOVEMENT;
        return false; // Implement based on your safety limits
    }
    
    private boolean isDrivingAggressively() {
        // Check if driver is making aggressive inputs
        // return Math.abs(driverController.getLeftY()) > 0.8 || 
        //        Math.abs(driverController.getRightX()) > 0.7;
        return false; // Implement based on your thresholds
    }
    
    private boolean isScoringState() {
        String state = elevManiSM.getCurrentState();
        return state.equals("L4Score") || state.equals("L3Score") || 
               state.equals("L2Score") || state.equals("L1Score");
    }
    
    private boolean wasJustScoring() {
        // Check if we were recently in a scoring state
        // Could track state history with a queue
        return false; // Implement if needed
    }
    
    private boolean elevatorIsAtSafeHeightForIntake() {
        // return elevManiSM.getElevatorHeight() <= Constants.Safety.MAX_INTAKE_HEIGHT;
        return true; // Implement based on your mechanism limits
    }
    
    // ===== PUBLIC API FOR COORDINATION =====
    
    public void setAutoMode(boolean auto) {
        isAutoMode = auto;
        if (auto) {
            // Special coordination for auto mode
            driveSM.requestState(DriveStateMachine.DriveState.FollowPath);
        }
    }
    
    public void setSafetyOverride(boolean override) {
        safetyOverride = override;
        if (override) {
            DataLogManager.log("Coordinator: SAFETY OVERRIDE ENABLED - Use caution!");
        }
    }
    
    public void executeAutoSequence(String sequenceName) {
        switch (sequenceName) {
            case "ScoreL4AndExit":
                executeScoreAndExitSequence();
                break;
            case "IntakeAndScore":
                executeIntakeAndScoreSequence();
                break;
            case "ClimbSequence":
                executeClimbSequence();
                break;
        }
    }
    
    // ===== PRE-DEFINED SEQUENCES =====
    
    private void executeScoreAndExitSequence() {
        DataLogManager.log("Coordinator: Starting ScoreAndExit sequence");
        
        // Sequence: Score -> Safe travel -> Drive away
        elevManiSM.requestState("L4Score");
        
        // Use a timer or state checking to sequence these actions
        // This would be more robust with proper state machine sequencing
    }
    
    private void executeIntakeAndScoreSequence() {
        DataLogManager.log("Coordinator: Starting IntakeAndScore sequence");
        
        // Sequence: Go to intake -> Wait for coral -> Score
        elevManiSM.requestState("IntakeCoral");
        driveSM.requestState(DriveStateMachine.DriveState.CoralStation);
        
        // This would use proper state monitoring and timing
    }
    
    private void executeClimbSequence() {
        DataLogManager.log("Coordinator: Starting Climb sequence");
        
        // Complex sequence involving both drive and elevator coordination
        elevManiSM.requestState("ClimbPrep");
        driveSM.requestState(DriveStateMachine.DriveState.ClimbRelative);
    }
    
    // ===== STATUS METHODS =====
    
    public boolean isSystemReady() {
        return !elevatorIsUnsafe() && 
               elevManiSM.getCurrentState().equals("SafeCoralTravel") &&
               driveSM.getCurrentState().equals(DriveStateMachine.DriveState.Teleop);
    }
    
    public boolean isScoringReady() {
        return isScoringState() && elevManiSM.hasCoral() && 
               driveSM.getCurrentState().equals(DriveStateMachine.DriveState.ReefAlign);
    }
    
    public String getSystemStatus() {
        if (elevatorIsUnsafe()) return "UNSAFE - Check elevator position";
        if (isScoringReady()) return "READY TO SCORE";
        if (elevManiSM.getCurrentState().equals("IntakeCoral")) return "INTAKING";
        return "NORMAL OPERATION";
    }
}