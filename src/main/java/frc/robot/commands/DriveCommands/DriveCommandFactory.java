package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveStateMachine;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.PoseEstimatorSubsystem;

/** Factory utilities for constructing drive state commands. */
public final class DriveCommandFactory {

    private DriveCommandFactory() {}

    public static Command createManualCommand(
            DriveStateMachine driveStateMachine,
            DriveSubsystem drive,
            PoseEstimatorSubsystem poseEstimator,
            XboxController driverController) {
        return new ManualDrive(driveStateMachine, drive, poseEstimator, driverController);
    }

    public static Command createFollowPathCommand(
            DriveStateMachine driveStateMachine,
            DriveSubsystem drive,
            PoseEstimatorSubsystem poseEstimator,
            XboxController driverController) {
        return new FollowPathDrive(drive, poseEstimator, driverController, driveStateMachine);
    }

    public static Command createBargeRelativeCommand(
            DriveStateMachine driveStateMachine,
            DriveSubsystem drive,
            PoseEstimatorSubsystem poseEstimator,
            XboxController driverController) {
        return new BargeRelativeDrive(drive, poseEstimator, driverController, driveStateMachine);
    }

    public static Command createClimbRelativeCommand(
            DriveStateMachine driveStateMachine,
            DriveSubsystem drive,
            PoseEstimatorSubsystem poseEstimator,
            XboxController driverController) {
        return new ClimbRelativeDrive(driveStateMachine, drive, poseEstimator, driverController);
    }

    public static Command createProcessorRelativeCommand(
            DriveStateMachine driveStateMachine,
            DriveSubsystem drive,
            PoseEstimatorSubsystem poseEstimator,
            XboxController driverController) {
        return new ProcessorRelativeDrive(driveStateMachine, drive, poseEstimator, driverController);
    }

    public static Command createCoralStationCommand(
            DriveStateMachine driveStateMachine,
            DriveSubsystem drive,
            PoseEstimatorSubsystem poseEstimator,
            XboxController driverController) {
        return new CoralStationDrive(driveStateMachine, drive, poseEstimator, driverController);
    }

    public static Command createReefRelativeCommand(
            DriveStateMachine driveStateMachine,
            DriveSubsystem drive,
            PoseEstimatorSubsystem poseEstimator,
            XboxController driverController) {
        return new ReefRelativeDrive(driveStateMachine, drive, poseEstimator, driverController);
    }

    public static Command createReefAlignCommand(
            DriveStateMachine driveStateMachine,
            DriveSubsystem drive,
            PoseEstimatorSubsystem poseEstimator,
            XboxController driverController) {
        return new ReefAlignDrive(drive, poseEstimator, driverController, driveStateMachine);
    }

    public static Command createCancelledCommand(DriveSubsystem drive) {
        return new CancelledDrive(drive);
    }
}
