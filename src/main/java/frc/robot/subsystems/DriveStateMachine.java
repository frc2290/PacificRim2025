package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.DriveSubsystem;
<<<<<<< HEAD
import frc.utils.FlytDashboardV2;
=======
>>>>>>> 6c8e26f (added stuff)
import frc.utils.PoseEstimatorSubsystem;
import frc.robot.commands.DriveCommands.*;

/**
 * Drive State Machine - Handles all drive behaviors and state transitions
 * Uses simple command switching rather than GraphCommand for drive states
 */
public class DriveStateMachine extends SubsystemBase {
<<<<<<< HEAD

=======
>>>>>>> 6c8e26f (added stuff)
    // Subsystems
    private final DriveSubsystem drive;
    private final PoseEstimatorSubsystem pose;
    private final XboxController driverController;
<<<<<<< HEAD
    private final FlytDashboardV2 dashboard;
=======
>>>>>>> 6c8e26f (added stuff)
    
    // State management
    private DriveState currentState = DriveState.Teleop;
    private DriveState goalState = DriveState.Teleop;
    private DriveState previousState = DriveState.Teleop;
    private Command currentCommand = null;
    private boolean isTransitioning = false;
    
    // State transition tracking
    private double stateChangeTime = 0;
    private static final double TRANSITION_TIMEOUT = 2.0; // seconds
    
    // Auto alignment settings
    private boolean rightReefAlignment = true; // true for right reef, false for left
    
    public DriveStateMachine(DriveSubsystem drive, PoseEstimatorSubsystem pose, 
<<<<<<< HEAD
        XboxController driverController) {
        this.drive = drive;
        this.pose = pose;
        this.driverController = driverController;
        this.dashboard = new FlytDashboardV2("DriveStateMachine");
        
=======
                           XboxController driverController) {
        this.drive = drive;
        this.pose = pose;
        this.driverController = driverController;
>>>>>>> 6c8e26f (added stuff)
    }
    
    /**
     * Request a drive state change
     */
    public void requestState(DriveState newState) {
        if (newState != goalState) {
            DataLogManager.log("DriveSM: Requesting state change from " + 
                              goalState + " to " + newState);
            goalState = newState;
        }
    }
    
    /**
     * Request a drive state change with alignment side
     */
    public void requestState(DriveState newState, boolean rightSide) {
        this.rightReefAlignment = rightSide;
        requestState(newState);
    }
    
    /**
     * Cancel current state and return to teleop
     */
    public void cancel() {
        requestState(DriveState.Teleop);
    }
    
    /**
     * Get current drive state
     */
    public DriveState getCurrentState() {
        return currentState;
    }
    
    /**
     * Get goal drive state
     */
    public DriveState getGoalState() {
        return goalState;
    }
    
    /**
     * Check if currently transitioning between states
     */
    public boolean isTransitioning() {
        return isTransitioning;
    }
    
    /**
     * Get alignment side preference
     */
    public boolean isRightReefAlignment() {
        return rightReefAlignment;
    }
    
    /**
     * Set alignment side preference
     */
    public void setAlignmentSide(boolean rightSide) {
        this.rightReefAlignment = rightSide;
    }
    
    /**
     * Check if at goal state
     */
    public boolean atGoalState() {
        return currentState == goalState;
    }
    
    /**
     * Check if drive system is ready for operation
     */
    public boolean isReady() {
        return !isTransitioning && currentState != DriveState.Cancelled;
    }
    
    /**
     * Main state machine update
     */
    @Override
    public void periodic() {
        if (!DriverStation.isEnabled()) {
            // Ensure drive is stopped when disabled
            if (currentCommand != null && currentCommand.isScheduled()) {
                currentCommand.cancel();
            }
            return;
        }
        
        handleStateTransitions();
        monitorStateTimeout();
        updateStateTracking();
    }
    
    /**
     * Handle state transitions
     */
    private void handleStateTransitions() {
        if (goalState != currentState) {
            // Start transition
            if (!isTransitioning) {
                startTransition();
            }
            
            // Complete transition when command is finished
            if (currentCommand != null && !currentCommand.isScheduled()) {
                completeTransition();
            }
        }
    }
    
    /**
     * Start a state transition
     */
    private void startTransition() {
        DataLogManager.log("DriveSM: Starting transition to " + goalState);
        isTransitioning = true;
        stateChangeTime = Timer.getFPGATimestamp();
        previousState = currentState;
        
        // Cancel current command if running
        if (currentCommand != null && currentCommand.isScheduled()) {
            currentCommand.cancel();
        }
        
        // Create and schedule new command based on state
        currentCommand = createCommandForState(goalState);
        if (currentCommand != null) {
            currentCommand.schedule();
        } else {
            // Fallback to teleop if command creation fails
            DataLogManager.log("DriveSM: Failed to create command for " + goalState + ", falling back to Teleop");
            goalState = DriveState.Teleop;
            currentCommand = createCommandForState(DriveState.Teleop);
            if (currentCommand != null) {
                currentCommand.schedule();
            }
        }
    }
    
    /**
     * Complete a state transition
     */
    private void completeTransition() {
        DataLogManager.log("DriveSM: Completed transition to " + goalState);
        currentState = goalState;
        isTransitioning = false;
        stateChangeTime = 0;
    }
    
    /**
     * Monitor for state transition timeouts
     */
    private void monitorStateTimeout() {
        if (isTransitioning && stateChangeTime > 0) {
            double elapsed = Timer.getFPGATimestamp() - stateChangeTime;
            if (elapsed > TRANSITION_TIMEOUT) {
                DataLogManager.log("DriveSM: Transition timeout from " + 
                                  previousState + " to " + goalState);
                // Cancel transition and return to previous state
                if (currentCommand != null && currentCommand.isScheduled()) {
                    currentCommand.cancel();
                }
                goalState = previousState;
                isTransitioning = false;
            }
        }
    }
    
    /**
     * Update state tracking and diagnostics
     */
    private void updateStateTracking() {
        // Add any additional state tracking logic here
        // For example, check if alignment states are achieving their goals
        
<<<<<<< HEAD
        // if (currentState == DriveState.ReefAlign && currentCommand instanceof ReefAlignDrive) {
        //     ReefAlignDrive alignCommand = (ReefAlignDrive) currentCommand;
        //     boolean isAligned = alignCommand.isAligned();
        //     // Could use this for LED feedback or other indicators
        // }
=======
        if (currentState == DriveState.ReefAlign && currentCommand instanceof ReefAlignDrive) {
            ReefAlignDrive alignCommand = (ReefAlignDrive) currentCommand;
            boolean isAligned = alignCommand.isAligned();
            // Could use this for LED feedback or other indicators
        }
>>>>>>> 6c8e26f (added stuff)
    }
    
    /**
     * Create appropriate command for the given drive state
     */
    private Command createCommandForState(DriveState state) {
        switch (state) {
            case Teleop:
                return new TeleopDrive(drive, pose, driverController, this);
                
            case ReefRelative:
                return new ReefRelativeDrive(drive, pose, driverController, this);
                
            case ReefAlign:
<<<<<<< HEAD
                return new  ReefRelativeDrive(drive, pose, driverController, this);
=======
                return new ReefAlignDrive(drive, pose, driverController, this, rightReefAlignment);
>>>>>>> 6c8e26f (added stuff)
                
            case CoralStation:
                return new CoralStationDrive(drive, pose, driverController, this);
                
            case ProcessorRelative:
                return new ProcessorRelativeDrive(drive, pose, driverController, this);
                
            case FollowPath:
                // This would typically be handled by auto commands
<<<<<<< HEAD
                return new FollowPathDrive(drive, pose, driverController, this);
=======
                return Commands.none().withName("FollowPathPlaceholder");
>>>>>>> 6c8e26f (added stuff)
                
            case BargeRelative:
                return new BargeRelativeDrive(drive, pose, driverController, this);
                
            case ClimbRelative:
                return new ClimbRelativeDrive(drive, pose, driverController, this);
                
            case Cancelled:
<<<<<<< HEAD
                return Commands.none(); // Return a no-op command for Cancelled
                
            default:
                DataLogManager.log("DriveSM: Unknown state " + state);
                return Commands.none(); // Return a no-op command for unknown states
=======
                return new CancelDrive(drive, this);
                
            default:
                DataLogManager.log("DriveSM: Unknown state " + state);
                return new TeleopDrive(drive, pose, driverController, this);
>>>>>>> 6c8e26f (added stuff)
        }
    }
    
    /**
     * Emergency stop - immediately cancel everything
     */
    public void emergencyStop() {
        DataLogManager.log("DriveSM: EMERGENCY STOP");
        if (currentCommand != null && currentCommand.isScheduled()) {
            currentCommand.cancel();
        }
<<<<<<< HEAD
        //drive.stop(); // Direct hardware control for safety
=======
        drive.stop(); // Direct hardware control for safety
>>>>>>> 6c8e26f (added stuff)
        currentState = DriveState.Cancelled;
        goalState = DriveState.Cancelled;
        isTransitioning = false;
    }
    
    /**
     * Reset to default state
     */
    public void reset() {
        DataLogManager.log("DriveSM: Reset to Teleop");
        if (currentCommand != null && currentCommand.isScheduled()) {
            currentCommand.cancel();
        }
        currentState = DriveState.Teleop;
        goalState = DriveState.Teleop;
        isTransitioning = false;
<<<<<<< HEAD
        //drive.stop(); // Ensure drive is stopped
=======
        drive.stop(); // Ensure drive is stopped
>>>>>>> 6c8e26f (added stuff)
    }
    
    /**
     * Drive State Enumeration
     */
    public enum DriveState {
        Teleop("Field-Oriented Teleop Drive"),
        ReefRelative("Reef-Relative Driving"),
        ReefAlign("Reef Alignment Hold"),
        CoralStation("Coral Station Facing"),
        ProcessorRelative("Processor-Relative Driving"),
        BargeRelative("Barge-Relative Driving"),
        ClimbRelative("Climb-Relative Driving"),
        FollowPath("Path Following"),
        Cancelled("Drive Cancelled");
        
        private final String description;
        
        DriveState(String description) {
            this.description = description;
        }
        
        public String getDescription() {
            return description;
        }
        
        @Override
        public String toString() {
            return name() + " (" + description + ")";
        }
    }
    
    // ===== Utility Methods for Drive Commands =====
    
    public boolean shouldLockRotation() {
        // Lock rotation in certain states for better control
        return currentState == DriveState.ReefAlign || 
               currentState == DriveState.CoralStation ||
               currentState == DriveState.ProcessorRelative;
    }
    
    public double getMaxSpeedMultiplier() {
        // Reduce speed in precision states
        switch (currentState) {
            case ReefAlign:
            case CoralStation:
                return 0.5; // 50% speed in precision states
            case Cancelled:
                return 0.0; // No movement when cancelled
            default:
                return 1.0; // Full speed otherwise
        }
    }
    
    public boolean isPrecisionMode() {
        return currentState == DriveState.ReefAlign || 
               currentState == DriveState.CoralStation;
    }
    
    /**
     * Get diagnostic information about current state
     */
    public String getDiagnostics() {
        return String.format("DriveSM: State=%s, Goal=%s, Transitioning=%s, Command=%s",
            currentState, goalState, isTransitioning,
            (currentCommand != null) ? currentCommand.getName() : "None");
    }
}