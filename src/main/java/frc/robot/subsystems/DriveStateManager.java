package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DriveCommands.CoralStationDrive;
import frc.robot.commands.DriveCommands.FollowPathDrive;
import frc.robot.commands.DriveCommands.ProcessorRelativeDrive;
import frc.robot.commands.DriveCommands.ReefAlignDrive;
import frc.robot.commands.DriveCommands.ReefRelativeDrive;
import frc.robot.commands.DriveCommands.TeleopDrive;
import frc.utils.PoseEstimatorSubsystem;

/**
 * Subsystem that owns the drive state machine logic.  This manager tracks the
 * current and goal drive states and schedules the appropriate command when a
 * transition is requested.
 */
public class DriveStateManager extends SubsystemBase {

    /**
     * Drive state options copied from the legacy manager.
     */
    public enum DriveState {
        Teleop,            // Field oriented freerome
        FollowPath,        // Auto path following
        BargeRelative,     // Faces barge
        ClimbRelative,     // Faces climb
        ProcessorRelative, // Faces processor
        CoralStation,      // Faces intake based on half field
        ReefRelative,      // Faces reef based on robot position
        ReefAlign,         // Locked to reef, holding position
        Cancelled
    }

    private final DriveSubsystem drive;
    private final PoseEstimatorSubsystem pose;
    private final XboxController driverController;

    private Command oldDriveCommmand = null;
    private Command currentDriveCommand = null;

    private DriveState driveState = DriveState.Teleop;
    private DriveState goalDriveState = DriveState.Teleop;

    private boolean rotLock = true;
    private boolean rightScore = false;

    public DriveStateManager(DriveSubsystem drive,
                             PoseEstimatorSubsystem pose,
                             XboxController driverController) {
        this.drive = drive;
        this.pose = pose;
        this.driverController = driverController;
    }

    /**
     * Run the drive state machine logic and schedule any newly selected
     * commands.
     */
    public void process() {
        switch (goalDriveState) {
            case Teleop:
                currentDriveCommand = new TeleopDrive(this, drive, pose, driverController);
                driveState = DriveState.Teleop;
                break;
            case FollowPath:
                currentDriveCommand = new FollowPathDrive(this, drive, pose, driverController);
                driveState = DriveState.FollowPath;
                break;
            case BargeRelative:
                // command to tell robot to face barge
                driveState = DriveState.BargeRelative;
                break;
            case ClimbRelative:
                // command to tell robot to face climb
                driveState = DriveState.ClimbRelative;
                break;
            case ProcessorRelative:
                currentDriveCommand = new ProcessorRelativeDrive(this, drive, pose, driverController);
                driveState = DriveState.ProcessorRelative;
                break;
            case CoralStation:
                currentDriveCommand = new CoralStationDrive(this, drive, pose, driverController);
                driveState = DriveState.CoralStation;
                break;
            case ReefRelative:
                currentDriveCommand = new ReefRelativeDrive(this, drive, pose, driverController);
                driveState = DriveState.ReefRelative;
                break;
            case ReefAlign:
                currentDriveCommand = new ReefAlignDrive(this, drive, pose, driverController);
                driveState = DriveState.ReefAlign;
                break;
            case Cancelled:
                // make sure all of the drive commands are cancelled and robot is stopped
                driveState = DriveState.Cancelled;
                currentDriveCommand = null;
                break;
            default:
                System.out.println("Unknown State!!!!!!!!!!");
                break;
        }

        if (oldDriveCommmand != currentDriveCommand && currentDriveCommand != null) {
            safeSwitch(oldDriveCommmand, currentDriveCommand).schedule();
            oldDriveCommmand = currentDriveCommand;
        }
    }

    /**
     * Request a new goal drive state.
     */
    public Command setGoalDriveCommand(DriveState newGoal) {
        return Commands.runOnce(() -> goalDriveState = newGoal);
    }

    /**
     * Force the current drive state without scheduling a transition command.
     */
    public Command setCurrentDriveStateCommand(DriveState curState) {
        return Commands.runOnce(() -> driveState = curState);
    }

    /**
     * Check whether driver rotation input is currently locked out.
     */
    public boolean getRotationLock() {
        return rotLock;
    }

    public void setRotationLock(boolean lock) {
        rotLock = lock;
    }

    public Command toggleRotationLock() {
        return Commands.runOnce(() -> rotLock = !rotLock);
    }

    /**
     * Query which branch is currently selected for scoring.
     */
    public boolean getRightScore() {
        return rightScore;
    }

    /**
     * Update which branch is currently selected for scoring.
     */
    public Command setRightScoreCommand(boolean right) {
        return Commands.runOnce(() -> rightScore = right);
    }

    private Command safeSwitch(Command oldCmd, Command newCmd) {
        return new InstantCommand(() -> {
            if (oldCmd != null && oldCmd.isScheduled()) {
                oldCmd.cancel();
            }
            if (newCmd != null) {
                newCmd.schedule();
            }
        });
    }

    /**
     * Wrapper preserved for backwards compatibility.
     */
}
