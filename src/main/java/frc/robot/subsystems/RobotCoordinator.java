package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.DriveStateManager.DriveState;

/**
 * Coordinates high level interactions between the arm and drive state
 * managers.  This class is intended to host cross-subsystem triggers that tie
 * the independent state machines together without re-introducing a monolithic
 * centralized manager.
 */
public class RobotCoordinator extends SubsystemBase {

    private final ArmStateManager armStateManager;
    private final DriveStateManager driveStateManager;

    public RobotCoordinator(ArmStateManager armStateManager, DriveStateManager driveStateManager) {
        this.armStateManager = armStateManager;
        this.driveStateManager = driveStateManager;

        configureInteractions();
    }

    /**
     * Set up simple cross-subsystem triggers.  As an example, when the arm has
     * finished moving to its target, the drive goal is updated based on whether
     * a coral is detected in the manipulator.
     */
    private void configureInteractions() {
        Trigger atGoal = armStateManager.atElevManiTarget();
        atGoal.and(armStateManager.hasCoralTrigger())
                .onTrue(driveStateManager.setGoalDriveCommand(DriveState.ReefAlign));
        atGoal.and(armStateManager.hasCoralTrigger().negate())
                .onTrue(driveStateManager.setGoalDriveCommand(DriveState.CoralStation));
    }
}
